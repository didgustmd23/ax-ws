"""D2 W4 입고 접수: 관측 → 입고 사건 대조 → 작업 자동 등록을 원장 한 트랜잭션으로 기록한다.

- 규칙은 receipt.py(순수)가 정하고, 이 모듈은 원장 읽기·쓰기와 registry 수용 검사를 묶는다.
- 관측 기록·사건 변경·등록 요청 기록·작업 생성은 같은 SQLite 트랜잭션이다. 중간에 실패하면 모두
  되돌리고 메모리 registry에도 반영하지 않으며, runtime을 고장 상태로 두어 이후 전송을 막는다.
- 등록은 QUEUED 작업 생성일 뿐 운반 완료가 아니다. 배정은 명시 입력(AssignRoles)으로 남는다.
- W5: 등록과 같은 트랜잭션에서 재고 항목(AT_SOURCE)을 만들거나 넘겨받고, 목적지 용량·물체 종류를 검사해
  예약을 결합한다. 이후 재고 갱신은 작업 판정 기록(task_store.record_decision)과 함께 일어난다.
- ROS Topic·Service를 만들지 않는다(운영 /ax/fleet/receipt/observations·/ax/fleet/tasks/register 미연결).
"""
import uuid
from dataclasses import dataclass

from . import group_model as gm
from . import inventory as inv
from . import receipt as rc
from .task_store import StoreConflict, StoreError

RECEIPT_EVENT_UNKNOWN = 'RECEIPT_EVENT_UNKNOWN'
RECEIPT_EVENT_NOT_REGISTRABLE = 'RECEIPT_EVENT_NOT_REGISTRABLE'


@dataclass(frozen=True)
class IntakeResult:
    """관측 하나의 접수 결과. decision: TASK_CREATED·DUPLICATE·PENDING·REJECTED·REJECT(관측 거부)."""

    decision: str
    reason_code: str = ''
    receipt_event_id: str = ''
    task_id: str = ''
    replayed: bool = False


# 등록 응답 사전(RegisterTask 응답 필드)
def _response(accepted, task_id='', reason_code=''):
    return {'accepted': accepted, 'task_id': task_id, 'reason_code': reason_code}


# 카탈로그 검사: 지도 revision·지점 revision이 registry와 같고, 입고 구역은 RECEIPT 용도
def _check_catalog(catalog, registry, config):
    if catalog.map_revision != registry.map_revision:
        raise gm.SpecError('MAP_REVISION_MISMATCH', 'catalog')

    if catalog.revisions() != registry.locations:
        raise gm.SpecError('LOCATION_REVISION_MISMATCH', 'catalog')

    for zone in config.receipt_zones:
        if catalog.get(zone)['purpose'] != 'RECEIPT':
            raise gm.SpecError('INVALID_CONFIG', f'입고 구역 용도가 RECEIPT가 아님: {zone}')


class ReceiptIntake:
    """입고 관측·작업 등록 접수. runtime의 원장·registry·시계를 공유한다."""

    # 설정·카탈로그를 등록 지점과 대조하고 runtime에 연결한다. 카탈로그는 원장에 기록한다
    def __init__(self, runtime, config, catalog, uuid_factory=None):
        rc.validate_config(config, runtime.registry.locations)
        _check_catalog(catalog, runtime.registry, config)

        self.runtime = runtime
        self.store = runtime.store
        self.registry = runtime.registry
        self.config = config
        self.catalog = catalog
        self.new_uuid = uuid_factory or (lambda: str(uuid.uuid4()))
        self.store.sync_locations(catalog.map_revision, catalog.locations)

    # 원장 쓰기 실패: runtime 고장으로 이후 전송을 막는다(메모리 registry는 바꾸지 않았다)
    def _fault(self, error):
        self.runtime.fault = f'STORE_WRITE_FAILED:{error}'
        self.runtime.notes.append(('INTAKE_STORE_FAULT', str(error)))

    # 입고 관측 하나 접수
    def observe(self, data):
        try:
            obs = rc.parse_observation(data)
        except gm.SpecError as error:
            return IntakeResult('REJECT', error.code)

        if self.runtime.fault:
            return IntakeResult('REJECT', 'RUNTIME_STORE_FAULT')

        now = self.runtime.clock()

        # 시각이 무효면 기록하지 않는다(시각 회복 뒤 같은 관측 재전송을 판정할 수 있게)
        if not now.time_valid:
            return IntakeResult('REJECT', 'TIME_INVALID')

        created = []
        try:
            with self.store.transaction():
                result = self._observe_locked(obs, now, created)
        except StoreError as error:
            self._fault(error)
            return IntakeResult('REJECT', 'STORE_WRITE_FAILED')

        # 원장 기록이 끝난 작업만 registry에 반영한다
        for state in created:
            self.registry.commit(state.spec.task_id, state)

        return result

    # 관측 판정과 기록(트랜잭션 안)
    def _observe_locked(self, obs, now, created):
        obs_digest = rc.observation_digest(obs)
        prior = self.store.receipt_observation(obs.observation_id)

        # 같은 관측 ID 재전송: 같은 내용이면 처음 판정, 다르면 거부(기록 덮어쓰지 않음)
        if prior is not None:
            if prior['digest'] != obs_digest:
                return IntakeResult('REJECT', rc.DUPLICATE_REQUEST, prior['receipt_event_id'])

            event = (self.store.receipt_event(prior['receipt_event_id'])
                     if prior['receipt_event_id'] else None)
            return IntakeResult(prior['decision'], prior['reason_code'],
                                prior['receipt_event_id'], event['task_id'] if event else '',
                                replayed=True)

        reason = rc.observation_rejection(obs, now, self.config, self.registry.mode)
        if reason:
            return self._record(obs, obs_digest, 'REJECT', reason)

        provenance = rc.PROVENANCE_BY_SOURCE[obs.source]

        # 식별 불명: 입고 구역마다 대기 사건 1개에 관측 ID를 덧붙인다(물체 ID와 합치지 않음, 작업 없음).
        # 관측 ID 상한을 넘으면 관측 기록에만 남기고 사건은 그대로 둔다
        if not rc.identified(obs, self.config):
            event = self.store.pending_receipt_event(obs.zone_id,
                                                     rc.RECEIPT_PENDING_IDENTIFICATION)
            if event is None:
                event = self._new_event('', obs, 0, rc.RECEIPT_PENDING_IDENTIFICATION,
                                        provenance)
                self.store.put_receipt_event(event)
            elif len(event['observation_ids']) < self.config.max_observation_ids:
                event['observation_ids'].append(obs.observation_id)
                self.store.put_receipt_event(event)

            return self._record(obs, obs_digest, 'PENDING', '', event)

        events = self.store.receipt_events_for_object(obs.object_id)
        verdict = rc.classify_identified(obs, events, self.registry.states)

        if verdict.action == 'REJECT':
            return self._record(obs, obs_digest, 'REJECT', verdict.reason_code)

        # 반복 관측: 열린 사건에 관측 ID만 덧붙인다(상한 초과분은 관측 기록에만 남음)
        if verdict.action == 'DUPLICATE':
            event = self.store.receipt_event(verdict.receipt_event_id)
            if len(event['observation_ids']) < self.config.max_observation_ids:
                event['observation_ids'].append(obs.observation_id)
                self.store.put_receipt_event(event)
            return self._record(obs, obs_digest, 'DUPLICATE', '', event)

        # 재입고 미확인: 새 회차 사건을 거절 상태로만 남긴다(작업 0)
        if verdict.action == 'RERECEIPT':
            event = self._new_event(obs.object_id, obs, verdict.receipt_round,
                                    rc.RECEIPT_REJECTED, provenance, verdict.reason_code)
            self.store.put_receipt_event(event)
            return self._record(obs, obs_digest, 'REJECTED', verdict.reason_code, event)

        # 신규 입고: 사건을 만들고 같은 트랜잭션에서 내부 RegisterTask로 작업을 등록한다
        event = self._new_event(obs.object_id, obs, verdict.receipt_round,
                                rc.RECEIPT_TASK_CREATED, provenance)
        request = {
            'request_id': rc.receipt_request_id(event['receipt_event_id']),
            'receipt_event_id': event['receipt_event_id'],
            'object_id': obs.object_id,
            'source_location_id': obs.zone_id,
            'destination_location_id': '',
        }
        response = self._register_locked(request, event, created)

        if response['accepted']:
            event['task_id'] = response['task_id']
        else:
            event['state'] = rc.RECEIPT_REJECTED
            event['reason_code'] = response['reason_code']

        self.store.put_receipt_event(event)
        decision = 'TASK_CREATED' if response['accepted'] else 'REJECTED'

        return self._record(obs, obs_digest, decision, event['reason_code'], event)

    # 새 입고 사건 사전
    def _new_event(self, object_id, obs, receipt_round, state, provenance, reason_code=''):
        return {
            'receipt_event_id': f'rcpt-{self.new_uuid()}',
            'object_id': object_id,
            'zone_id': obs.zone_id,
            'receipt_round': receipt_round,
            'state': state,
            'task_id': '',
            'observation_ids': [obs.observation_id],
            'provenance': provenance,
            'reason_code': reason_code,
            'object_class': obs.object_class if object_id else '',
        }

    # 관측 판정 기록과 결과
    def _record(self, obs, obs_digest, decision, reason_code, event=None):
        event_id = event['receipt_event_id'] if event else ''
        self.store.insert_receipt_observation(obs.observation_id, obs_digest, event_id,
                                              decision, reason_code)

        return IntakeResult(decision, reason_code, event_id, event['task_id'] if event else '')

    # 작업 등록 요청 접수(RegisterTask 필드). 응답은 accepted·task_id·reason_code
    def register_task(self, data):
        try:
            request = rc.parse_request(data)
        except gm.SpecError as error:
            return _response(False, '', error.code)

        if self.runtime.fault:
            return _response(False, '', 'RUNTIME_STORE_FAULT')

        created = []
        try:
            with self.store.transaction():
                response = self._register_locked(request, None, created)
        except StoreError as error:
            self._fault(error)
            return _response(False, '', 'STORE_WRITE_FAILED')

        for state in created:
            self.registry.commit(state.spec.task_id, state)

        return response

    # 등록 판정과 기록(트랜잭션 안). event가 있으면 같은 트랜잭션에서 막 만든 입고 사건이다
    def _register_locked(self, request, event, created):
        request_digest = rc.digest(request)
        prior = self.store.register_request(request['request_id'])

        # 같은 요청 ID: 같은 내용이면 저장한 응답, 다르면 거절(기록 덮어쓰지 않음)
        if prior is not None:
            if prior['request_digest'] == request_digest:
                return prior['response']
            return _response(False, '', rc.DUPLICATE_REQUEST)

        response = self._decide_registration(request, event, created)
        self.store.insert_register_request(request['request_id'], request_digest, response)

        return response

    # 등록 수락 여부를 정하고, 수락이면 작업 행을 만든다
    def _decide_registration(self, request, event, created):
        if event is None and request['receipt_event_id']:
            stored = self.store.receipt_event(request['receipt_event_id'])
            if stored is None:
                return _response(False, '', RECEIPT_EVENT_UNKNOWN)

            # 입고 사건당 작업 1개: 이미 작업이 있으면 그 작업 ID와 함께 거절
            if stored['task_id']:
                return _response(False, stored['task_id'], rc.DUPLICATE_REQUEST)

            return _response(False, '', RECEIPT_EVENT_NOT_REGISTRABLE)

        source, destination, reason = rc.resolve_locations(request, event, self.config)
        if reason:
            return _response(False, '', reason)

        entry = self.store.inventory_entry(request['object_id'])
        spec = self._build_spec(request, event, source, destination,
                                entry['revision'] + 1 if entry else 1)
        try:
            self.registry.check_admission(spec)
            state = gm.new_group_state(spec, self.registry.mode)
        except gm.SpecError as error:
            code = rc.OBJECT_BUSY if error.code == 'OBJECT_IN_ACTIVE_TASK' else error.code
            return _response(False, '', code)

        object_class = event['object_class'] if event else (entry['object_class'] if entry else '')
        reason = self._inventory_rejection(entry, source, destination, object_class)
        if reason:
            return _response(False, '', reason)

        # 재고 항목(출발 지점) + 목적지 예약 + 작업 행: 같은 트랜잭션
        revision = self.store.put_inventory_entry({
            'object_id': spec.object_id,
            'receipt_event_id': event['receipt_event_id'] if event else (
                entry['receipt_event_id'] if entry else ''),
            'object_class': object_class,
            'location_id': source,
            'carrier_robot_id': '',
            'assignment_generation': 0,
            'state': gm.INV_AT_SOURCE,
            'isolated': False,
            'task_id': spec.task_id,
        })
        # 항목 revision이 작업에 넣은 값과 다르면 트랜잭션 전체를 되돌린다(방어)
        if revision != spec.inventory_revision:
            raise StoreConflict(f'재고 revision 불일치: {revision} != {spec.inventory_revision}')

        self.store.insert_reservation(f'rsv-{self.new_uuid()}', destination, spec.task_id,
                                      spec.object_id)
        self.store.insert_task_row(state)
        created.append(state)

        return _response(True, spec.task_id, '')

    # 재고·목적지 검사(K7·K3): 기존 항목은 출발 지점에 확정 상태로 있어야 하고, 목적지는 사용 중이며
    # 물체 종류를 허용하고 용량(도착 물품 + 활성 예약)이 남아 있어야 한다
    def _inventory_rejection(self, entry, source, destination, object_class):
        if entry is not None and (
                entry['isolated'] or entry['location_id'] != source
                or entry['state'] not in (gm.INV_AT_SOURCE, gm.INV_AT_DESTINATION)):
            return inv.INVENTORY_MISMATCH

        info = self.catalog.get(destination)
        if not info['active']:
            return inv.LOCATION_INACTIVE_CODE

        if not inv.class_allowed(info, object_class):
            return inv.OBJECT_CLASS_NOT_ALLOWED

        items, active = self.store.location_load(destination)
        if items + active >= info['capacity']:
            return inv.LOCATION_FULL

        return ''

    # 표준 운반 계획 작업 정의. ID는 새로 발급하고 revision은 registry 명시값을 쓴다
    def _build_spec(self, request, event, source, destination, inventory_revision):
        if event is not None:
            input_source_id = f'd2_receipt:{event["receipt_event_id"]}'
        else:
            input_source_id = f'd2_register:{request["request_id"]}'[:gm.MAX_ID_LENGTH]

        return gm.GroupTaskSpec(
            team_id=self.config.team_id,
            task_id=f'task-{self.new_uuid()}',
            object_id=request['object_id'],
            inventory_revision=inventory_revision,
            source_location_id=source,
            source_location_revision=self.registry.locations[source]
            if source in self.registry.locations else 0,
            destination_location_id=destination,
            destination_location_revision=self.registry.locations[destination]
            if destination in self.registry.locations else 0,
            map_revision=self.registry.map_revision,
            root_execution_id=f'exec-{self.new_uuid()}',
            task_revision=1,
            schedule_revision=0,
            priority=self.config.priority,
            source_transfer_id=f'xfer-{self.new_uuid()}',
            destination_transfer_id=f'xfer-{self.new_uuid()}',
            input_source_id=input_source_id,
            steps=gm.standard_transfer_plan(),
        )
