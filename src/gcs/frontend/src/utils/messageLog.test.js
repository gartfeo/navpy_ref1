import { describe, it, expect } from 'vitest';
import {
  appendEntries,
  mergeBacklog,
  newestKey,
  filterEntries,
  severityTone,
  formatExport,
  namedValueRows,
  NAMED_VALUE_STALE_S,
} from './messageLog';

const text = (seq, severity = 6, t = `m${seq}`, epoch = 'e1') => ({
  seq, epoch, kind: 'text', ts: 1000 + seq, source: 'autopilot', severity, label: 'INFO', text: t,
});
const value = (seq) => ({
  seq, epoch: 'e1', kind: 'value', ts: 1000 + seq, source: 'autopilot', severity: 6, label: 'VALUE',
  name: 'NAVWD', value: 1, text: 'NAVWD = 1',
});

describe('appendEntries', () => {
  it('appends live entries and keeps the newest within capacity', () => {
    const h = appendEntries([text(1), text(2)], [text(3), text(4)], 3);
    expect(h.map((e) => e.seq)).toEqual([2, 3, 4]);
  });

  it('returns the same history when nothing is new', () => {
    const h = [text(1)];
    expect(appendEntries(h, [])).toBe(h);
  });

  // The backend log starts over (a reconnect or restart): a new epoch, seq from 1.
  it('restarts the history on a new epoch', () => {
    const h = appendEntries([text(7), text(8)], [text(1, 6, 'new', 'e2')]);
    expect(h.map((e) => [e.epoch, e.seq])).toEqual([['e2', 1]]);
  });

  // The backlog can hold entries the broadcast has not drained yet: the live
  // feed then repeats them. An overlap is not a restart.
  it('drops entries it already has and keeps the older ones', () => {
    const h = appendEntries([text(1), text(2), text(3)], [text(2), text(3), text(4)]);
    expect(h.map((e) => e.seq)).toEqual([1, 2, 3, 4]);
    const same = [text(1), text(2), text(3)];
    expect(appendEntries(same, [text(2), text(3)])).toBe(same);
  });
});

describe('mergeBacklog', () => {
  it('keeps live entries of its epoch newer than the backlog, once', () => {
    const merged = mergeBacklog([text(1), text(2), text(3)], 'e1', [text(3), text(4)]);
    expect(merged.map((e) => e.seq)).toEqual([1, 2, 3, 4]);
  });

  it("drops a previous log's history", () => {
    const merged = mergeBacklog([text(1, 6, 'a', 'e2')], 'e2', [text(5), text(6)]);
    expect(merged.map((e) => [e.epoch, e.seq])).toEqual([['e2', 1]]);
    expect(mergeBacklog([], 'e2', [text(5)])).toEqual([]);
  });
});

describe('newestKey', () => {
  it('names the newest entry, so a same-length history still changes it', () => {
    expect(newestKey([])).toBe('');
    expect(newestKey([text(1), text(2)])).toBe('e1:2');
    expect(newestKey([text(2), text(3)])).not.toBe(newestKey([text(1), text(2)]));
  });
});

describe('filterEntries', () => {
  const entries = [text(1, 6), text(2, 4), text(3, 2), value(4)];
  it('warnings keeps WARN and worse texts', () => {
    expect(filterEntries(entries, 'warnings').map((e) => e.seq)).toEqual([2, 3]);
  });
  it('texts and values split by kind', () => {
    expect(filterEntries(entries, 'texts').map((e) => e.seq)).toEqual([1, 2, 3]);
    expect(filterEntries(entries, 'values').map((e) => e.seq)).toEqual([4]);
    expect(filterEntries(entries, 'all')).toHaveLength(4);
  });
});

describe('severityTone', () => {
  it('maps MAV_SEVERITY to row colours', () => {
    expect(severityTone(text(1, 2))).toBe('error');
    expect(severityTone(text(1, 3))).toBe('error');
    expect(severityTone(text(1, 4))).toBe('warning');
    expect(severityTone(text(1, 6))).toBe('info');
    expect(severityTone(value(1))).toBe('info');
  });
});

describe('formatExport', () => {
  it('writes one tab-separated line per entry', () => {
    const out = formatExport([text(1, 6, 'Armed')], 'UAV 1');
    const fields = out.trim().split('\t');
    expect(fields.slice(1)).toEqual(['UAV 1', 'autopilot', 'INFO', 'Armed']);
    expect(fields[0]).toMatch(/^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}$/);
    expect(out.endsWith('\n')).toBe(true);
  });
});

describe('namedValueRows', () => {
  it('sorts by name and flags a value not heard for a while', () => {
    const now = 100_000;
    const rows = namedValueRows({
      NAVWD: { value: 1, source: 'autopilot', ts: now / 1000 - (NAMED_VALUE_STALE_S + 1) },
      ALT: { value: 2, source: 'companion', ts: now / 1000 - 1 },
    }, now);
    expect(rows.map((r) => [r.name, r.stale])).toEqual([['ALT', false], ['NAVWD', true]]);
  });
});
