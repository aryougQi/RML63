#!/usr/bin/env python3
"""Durable M20 transactions and the 1 -> 2 placement cursor; no robot commands."""
import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import sqlite3
import time


DEFAULT_PATH = '~/.ros/fixed_grasp/m20_transactions.sqlite3'
TEST_PATH = '~/.ros/fixed_grasp/m20_test_transactions.sqlite3'


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


class Ledger:
    def __init__(self, path):
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lease = open(str(self.path)+'.lock', 'a')
        try:
            fcntl.flock(self.lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception:
            self.lease.close()
            raise RuntimeError('M20 ledger is in use; stop the old arm server first')
        self.db = sqlite3.connect(str(self.path), check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.executescript('''
            CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value INTEGER NOT NULL);
            INSERT OR IGNORE INTO meta VALUES ('cursor', 0);
            INSERT OR IGNORE INTO meta VALUES ('generation', 0);
            CREATE TABLE IF NOT EXISTS transactions (
                tx TEXT PRIMARY KEY, signature TEXT NOT NULL, request TEXT NOT NULL,
                block INTEGER NOT NULL, status TEXT NOT NULL, task_success INTEGER NOT NULL DEFAULT 0,
                error_code TEXT NOT NULL, message TEXT NOT NULL, stage TEXT NOT NULL DEFAULT '', updated REAL NOT NULL,
                generation INTEGER NOT NULL DEFAULT 0);
            CREATE TABLE IF NOT EXISTS requests (
                id TEXT PRIMARY KEY, signature TEXT NOT NULL, tx TEXT NOT NULL, operation TEXT NOT NULL);
        ''')
        if 'generation' not in {row[1] for row in self.db.execute('PRAGMA table_info(transactions)')}:
            self.db.execute('ALTER TABLE transactions ADD COLUMN generation INTEGER NOT NULL DEFAULT 0')
        # A crash between durable acceptance and a terminal result is ambiguous.
        with self.db:
            self.db.execute("UPDATE transactions SET status='unknown', task_success=0, error_code='RESULT_UNKNOWN', "
                            "message='Arm process restarted; verify physical outcome before resolving on M20', updated=? "
                            "WHERE status IN ('accepted','running')", (time.time(),))

    @property
    def cursor(self):
        return self.db.execute("SELECT value FROM meta WHERE key='cursor'").fetchone()[0]

    @property
    def generation(self):
        return self.db.execute("SELECT value FROM meta WHERE key='generation'").fetchone()[0]

    def transaction(self, tx):
        row = self.db.execute('SELECT * FROM transactions WHERE tx=?', (tx,)).fetchone()
        return dict(row) if row else None

    def latest(self):
        row = self.db.execute('SELECT * FROM transactions ORDER BY updated DESC LIMIT 1').fetchone()
        return dict(row) if row else None

    def request(self, request_id):
        row = self.db.execute('SELECT * FROM requests WHERE id=?', (request_id,)).fetchone()
        return dict(row) if row else None

    def remember_request(self, request):
        with self.db:
            self.db.execute('INSERT OR IGNORE INTO requests VALUES (?,?,?,?)',
                (request['request_id'], digest(request), request['transaction_id'], request['operation']))

    def accept(self, request, signature):
        # This commit MUST complete before Server.goal can dispatch any command.
        with self.db:
            block = (1, 2)[self.cursor]
            self.db.execute('INSERT INTO transactions (tx,signature,request,block,status,error_code,message,updated,generation) '
                            'VALUES (?,?,?,?,?,?,?,?,?)', (request['transaction_id'], signature, canonical(request),
                            block, 'accepted', 'OK', 'Accepted place for block %d' % block, time.time(), self.generation))
            self.db.execute('INSERT INTO requests VALUES (?,?,?,?)',
                (request['request_id'], digest(request), request['transaction_id'], request['operation']))
        return self.transaction(request['transaction_id'])

    def progress(self, tx, stage):
        with self.db:
            self.db.execute("UPDATE transactions SET status='running', stage=?, message=?, updated=? WHERE tx=?",
                            (stage, stage, time.time(), tx))

    def finish(self, tx, status, success, code, message, stage=''):
        record = self.transaction(tx)
        if record is None or record['status'] not in ('accepted', 'running'):
            return
        with self.db:
            self.db.execute('UPDATE transactions SET status=?, task_success=?, error_code=?, message=?, stage=?, updated=? WHERE tx=?',
                (status, int(success), code, message, stage, time.time(), tx))
            if success:
                # Failed/cancelled/unknown jobs never consume the next block.
                if self.cursor != record['block']-1 or self.generation != record['generation']:
                    raise RuntimeError('placement sequence conflicts with durable transaction')
                self.db.execute("UPDATE meta SET value=value+1 WHERE key='cursor'")

    def reset_sequence(self):
        with self.db:
            self.db.execute("UPDATE meta SET value=0 WHERE key='cursor'")
            self.db.execute("UPDATE meta SET value=value+1 WHERE key='generation'")
        # Preserve every transaction/request so old packets can never replay work.

    def confirm_completed(self, tx, source):
        record = self.transaction(tx)
        if not record or record['status'] not in ('unknown', 'failed', 'cancelled'):
            raise ValueError('only unknown/failed/cancelled transactions can be manually reconciled')
        if not source.strip() or self.cursor != record['block']-1 or self.generation != record['generation']:
            raise ValueError('verification source and matching current sequence are required')
        with self.db:
            self.db.execute("UPDATE transactions SET status='succeeded', task_success=1, error_code='OPERATOR_CONFIRMED', "
                            "message=?, updated=? WHERE tx=?", ('Operator verified completed: '+source, time.time(), tx))
            self.db.execute("UPDATE meta SET value=value+1 WHERE key='cursor'")

    def close(self):
        self.db.close()
        self.lease.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--database', default=DEFAULT_PATH)
    parser.add_argument('--reset-sequence', action='store_true')
    parser.add_argument('--confirm-reloaded', action='store_true', help='确认旧服务已停止且两个方块已补回原示教位置')
    parser.add_argument('--resolve-completed', metavar='TRANSACTION_ID', help='人工核实未知/失败事务其实已完成，不执行动作')
    parser.add_argument('--confirm-completed', action='store_true')
    parser.add_argument('--source', default='')
    args = parser.parse_args()
    if args.reset_sequence and not args.confirm_reloaded:
        parser.error('reset requires --confirm-reloaded; old transactions will remain deduplicated')
    if args.resolve_completed and (not args.confirm_completed or not args.source.strip() or args.reset_sequence):
        parser.error('resolve requires --confirm-completed --source and cannot be combined with reset')
    ledger = Ledger(args.database)
    try:
        if args.reset_sequence:
            ledger.reset_sequence()
        if args.resolve_completed:
            ledger.confirm_completed(args.resolve_completed, args.source)
        print(canonical(dict(next_grasp=(1, 2)[ledger.cursor] if ledger.cursor < 2 else None,
                             last_transaction=ledger.latest(), database=str(ledger.path))))
    finally:
        ledger.close()


if __name__ == '__main__': main()
