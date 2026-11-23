#!/usr/bin/env python3
"""D1 local durable execution ledger. Does not grant permission or drive actuators."""
import json
import math
import sqlite3
import uuid


class ExecutionLedger:
    """Commit before transmission; unresolved executions keep the robot reserved."""

    TERMINAL = {'REJECTED', 'ABORTED', 'CANCELED', 'ARRIVED'}

    def __init__(self, path):
        self.db = sqlite3.connect(str(path), isolation_level=None)
        self.db.row_factory = sqlite3.Row
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('''CREATE TABLE IF NOT EXISTS executions (
            execution_id TEXT PRIMARY KEY, robot TEXT NOT NULL,
            request TEXT NOT NULL, goal_uuid TEXT NOT NULL UNIQUE,
            state TEXT NOT NULL, detail TEXT NOT NULL DEFAULT '')''')
        self.db.execute('''CREATE TABLE IF NOT EXISTS holds (
            robot TEXT PRIMARY KEY, execution_id TEXT NOT NULL, reason TEXT NOT NULL)''')

    def close(self):
        self.db.close()

    def get(self, execution_id):
        row = self.db.execute('SELECT * FROM executions WHERE execution_id=?',
                              (execution_id,)).fetchone()
        return dict(row) if row else None

    def hold(self, robot, execution_id, reason):
        # First cause survives cancel/result updates and process restart.
        self.db.execute('INSERT OR IGNORE INTO holds VALUES (?,?,?)',
                        (robot, execution_id, reason))

    def held(self, robot):
        row = self.db.execute('SELECT * FROM holds WHERE robot=?', (robot,)).fetchone()
        return dict(row) if row else None

    def reserve(self, execution_id, robot, assignment_id, assignment_generation, x, y, yaw):
        if not execution_id or not assignment_id or robot not in (
                'robot_1', 'robot_2', 'robot_3', 'robot_4'):
            raise ValueError('Execution ID, assignment ID and supported robot required')
        if (type(assignment_generation) is not int or assignment_generation < 0
                or not all(math.isfinite(v) for v in (x, y, yaw))):
            raise ValueError('Finite map goal and nonnegative assignment generation required')
        request = json.dumps(dict(assignment_id=assignment_id,
            assignment_generation=assignment_generation, frame='map',
            x=float(x), y=float(y), yaw=float(yaw)), sort_keys=True, allow_nan=False)
        self.db.execute('BEGIN IMMEDIATE')
        try:
            old = self.get(execution_id)
            if old:
                if old['robot'] != robot or old['request'] != request:
                    raise ValueError('Execution ID already bound to different arguments')
                self.db.execute('COMMIT')
                return old, False
            if self.held(robot):
                raise ValueError('Robot held; input recovery does not authorize resumption')
            active = self.db.execute("SELECT execution_id FROM executions WHERE robot=? "
                "AND state NOT IN ('REJECTED','ABORTED','CANCELED','ARRIVED')", (robot,)).fetchone()
            if active:
                raise ValueError('Robot has an unresolved execution: ' + active[0])
            self.db.execute('INSERT INTO executions VALUES (?,?,?,?,?,?)',
                (execution_id, robot, request, uuid.uuid4().hex, 'PREPARED', ''))
            self.db.execute('COMMIT')
        except BaseException:
            self.db.execute('ROLLBACK')
            raise
        return self.get(execution_id), True

    def transition(self, execution_id, expected, state, detail=''):
        cursor = self.db.execute('UPDATE executions SET state=?, detail=? '
            'WHERE execution_id=? AND state=?', (state, detail, execution_id, expected))
        if cursor.rowcount != 1:
            raise ValueError('Unexpected or missing execution state')


class NavigationExecution:
    """Transport-independent state machine; callbacks carry the same execution ID."""

    def __init__(self, ledger):
        self.ledger = ledger

    def submit(self, send, **request):
        row, created = self.ledger.reserve(**request)
        if not created:
            return row  # Never replay even PREPARED after a process interruption.
        key = row['execution_id']
        self.ledger.transition(key, 'PREPARED', 'SENDING')
        try:
            send(row)
        except Exception as error:
            self.ledger.transition(key, 'SENDING', 'UNKNOWN', str(error))
            raise
        return self.ledger.get(key)

    def accepted(self, execution_id, accepted):
        self.ledger.transition(execution_id, 'SENDING', 'ACTIVE' if accepted else 'REJECTED')

    def cancel(self, execution_id, send_cancel):
        row = self.ledger.get(execution_id)
        if not row or row['state'] != 'ACTIVE':
            raise ValueError('Only the active owned goal can be canceled')
        self.ledger.transition(execution_id, 'ACTIVE', 'CANCEL_REQUESTED')
        try:
            send_cancel(row)
        except Exception as error:
            self.ledger.transition(execution_id, 'CANCEL_REQUESTED', 'UNKNOWN', str(error))
            raise

    def cancel_response(self, execution_id, accepted):
        # Cancellation acknowledgment is not a terminal Action result or actual stop.
        if not accepted:
            self.ledger.transition(execution_id, 'CANCEL_REQUESTED', 'ACTIVE', 'CANCEL_REJECTED')

    def result(self, execution_id, status):
        row = self.ledger.get(execution_id)
        if not row or row['state'] not in ('ACTIVE', 'CANCEL_REQUESTED'):
            raise ValueError('Result does not belong to an active owned execution')
        state = {4: 'ARRIVED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, 'UNKNOWN')
        self.ledger.transition(execution_id, row['state'], state, 'NAV2_STATUS_' + str(status))
