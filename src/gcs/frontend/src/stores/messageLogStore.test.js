import { describe, it, expect, beforeEach } from 'vitest';
import messageLogStore from './messageLogStore';

const entry = (seq, epoch) => ({ seq, epoch, kind: 'text', ts: 1, source: 'autopilot', severity: 4, label: 'WARN', text: `${epoch}:${seq}` });

beforeEach(() => messageLogStore.reset());

describe('messageLogStore.loadBacklog', () => {
  // A reply that was in flight when the UAV reconnected (new epoch).
  it('drops a backlog of an epoch other than the current one', () => {
    messageLogStore.ingest(1, [entry(1, 'e2')]);
    messageLogStore.loadBacklog(1, [], 'e1', 'e2');
    expect(messageLogStore.getEntries(1).map((e) => e.text)).toEqual(['e2:1']);
  });

  it('loads a backlog of the current epoch', () => {
    messageLogStore.ingest(1, [entry(2, 'e2')]);
    messageLogStore.loadBacklog(1, [entry(1, 'e2')], 'e2', 'e2');
    expect(messageLogStore.getEntries(1).map((e) => e.text)).toEqual(['e2:1', 'e2:2']);
  });
});
