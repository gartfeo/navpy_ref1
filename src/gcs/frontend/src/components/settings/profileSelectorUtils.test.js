import { describe, expect, it } from 'vitest';
import { visionProfileReply } from './profileSelectorUtils.js';

const answer = (status, json) => ({ ok: status >= 200 && status < 300, status, json });

describe('visionProfileReply', () => {
  it('returns the catalog of an ok answer', async () => {
    await expect(visionProfileReply(answer(200, async () => ({ profiles: { sim: {} } }))))
      .resolves.toEqual({ profiles: { sim: {} } });
  });

  it('throws the detail of a refusal, so its body never becomes the catalog', async () => {
    const refusal = answer(501, async () => ({ detail: 'not supported in simulation: writes vision_profiles.json' }));
    await expect(visionProfileReply(refusal))
      .rejects.toThrow('not supported in simulation: writes vision_profiles.json');
  });

  it('throws the status when an error has no JSON detail', async () => {
    const html = answer(502, async () => { throw new Error('html'); });
    await expect(visionProfileReply(html)).rejects.toThrow('HTTP 502');
  });
});
