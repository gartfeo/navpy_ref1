// Covers only the NavPy patches to the vendored MediaMTX WHEP reader (see
// ./NOTICE): the WHEP session resource must be DELETEd exactly once, with the
// URL the server returned in the POST Location header, whether close() happens
// after the 201 or while the POST is still in flight — and nothing may
// reconnect afterwards. Upstream's signalling is otherwise untouched, so it is
// driven here through a fake fetch + fake RTCPeerConnection rather than
// re-asserted.
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import MediaMTXWebRTCReader from './reader';

const WHEP_URL = 'http://jetson.test:8889/tracking/whep';

// One offer SDP for both the codec probe and the real offer. It names the three
// probe codecs verbatim, so #supportsNonAdvertisedCodec short-circuits with
// "already present" instead of rewriting payload types.
const OFFER_SDP = [
  'v=0',
  'o=- 0 0 IN IP4 0.0.0.0',
  's=-',
  't=0 0',
  'a=ice-ufrag:abcd',
  'a=ice-pwd:efgh',
  'm=video 9 UDP/TLS/RTP/SAVPF 96',
  'a=rtpmap:96 H264/90000',
  'm=audio 9 UDP/TLS/RTP/SAVPF 111',
  'a=rtpmap:111 pcma/8000/2 multiopus/48000/6 L16/48000/2',
  '',
].join('\r\n');

const ANSWER_SDP = OFFER_SDP;

let peerConnections;
let fetchCalls;
let fetchInits;
let optionsImpl;
let postDeferred;

const deferred = () => {
  let resolve;
  const promise = new Promise((r) => { resolve = r; });
  return { promise, resolve };
};

// The reader's own peer connection is the only one that opens a data channel;
// the three codec probes do not.
const readerPc = () => peerConnections.find((pc) => pc.dataChannels > 0);

const callsWithMethod = (method) => fetchCalls.filter((c) => c.method === method);

class FakePeerConnection {
  constructor() {
    this.connectionState = 'new';
    this.closed = false;
    this.dataChannels = 0;
    this.remoteDescriptions = 0;
    this.onicecandidate = null;
    this.onconnectionstatechange = null;
    this.ontrack = null;
    this.ondatachannel = null;
    peerConnections.push(this);
  }

  addTransceiver() {}

  createDataChannel() {
    this.dataChannels += 1;
    return { close() {} };
  }

  createOffer() {
    return Promise.resolve({ type: 'offer', sdp: OFFER_SDP });
  }

  setLocalDescription() {
    return Promise.resolve();
  }

  setRemoteDescription() {
    this.remoteDescriptions += 1;
    return Promise.resolve();
  }

  close() {
    this.closed = true;
  }
}

const postResponse = ({ status = 201, location = '/tracking/whep/session/abc' } = {}) => ({
  status,
  headers: { get: (name) => (name.toLowerCase() === 'location' ? location : null) },
  text: () => Promise.resolve(ANSWER_SDP),
  json: () => Promise.resolve({ error: 'rejected' }),
});

// Native promise continuations are not faked, so draining the microtask queue
// walks the whole signalling chain up to the pending POST.
const flush = async () => {
  for (let i = 0; i < 60; i++) await Promise.resolve();
};

beforeEach(() => {
  peerConnections = [];
  fetchCalls = [];
  fetchInits = [];
  optionsImpl = () => Promise.resolve({ status: 200, headers: { get: () => null } });
  postDeferred = deferred();
  vi.useFakeTimers();

  globalThis.RTCPeerConnection = FakePeerConnection;
  globalThis.RTCSessionDescription = class {
    constructor(init) { Object.assign(this, init); }
  };
  globalThis.fetch = vi.fn((url, opts = {}) => {
    const method = opts.method || 'GET';
    fetchCalls.push({ url, method, keepalive: opts.keepalive });
    fetchInits.push(opts);
    switch (method) {
      case 'OPTIONS':
        return optionsImpl();
      case 'POST':
        return postDeferred.promise;
      case 'PATCH':
        return Promise.resolve({ status: 204 });
      case 'DELETE':
        return Promise.resolve({ status: 200 });
      default:
        return Promise.reject(new Error(`unexpected ${method}`));
    }
  });
});

afterEach(() => {
  delete globalThis.RTCPeerConnection;
  delete globalThis.RTCSessionDescription;
  delete globalThis.fetch;
});

// Nothing may re-offer once the reader is closed.
const expectNoReconnect = async () => {
  const before = { options: callsWithMethod('OPTIONS').length, post: callsWithMethod('POST').length };
  vi.advanceTimersByTime(30_000);
  await flush();
  expect(callsWithMethod('OPTIONS')).toHaveLength(before.options);
  expect(callsWithMethod('POST')).toHaveLength(before.post);
};

describe('vendored WHEP reader session release', () => {
  it('deletes the session once, at the URL from the POST Location header', async () => {
    const onError = vi.fn();
    const reader = new MediaMTXWebRTCReader({ url: WHEP_URL, onError });

    await flush();
    postDeferred.resolve(postResponse());
    await flush();

    expect(callsWithMethod('POST')).toHaveLength(1);
    expect(callsWithMethod('DELETE')).toHaveLength(0);

    reader.close();
    await flush();

    expect(callsWithMethod('DELETE')).toEqual([{
      url: 'http://jetson.test:8889/tracking/whep/session/abc',
      method: 'DELETE',
      keepalive: true,
    }]);
    expect(readerPc().closed).toBe(true);

    // Idempotent: a second close (unmount after a pagehide, say) must not
    // re-delete a session that is already gone.
    reader.close();
    await flush();
    expect(callsWithMethod('DELETE')).toHaveLength(1);

    await expectNoReconnect();
    expect(onError).not.toHaveBeenCalled();
  });

  it('releases a session whose 201 arrives after close()', async () => {
    const onError = vi.fn();
    const reader = new MediaMTXWebRTCReader({ url: WHEP_URL, onError });

    await flush();
    expect(callsWithMethod('POST')).toHaveLength(1);

    // Close while the POST is in flight: there is no session URL to delete yet.
    reader.close();
    await flush();
    expect(callsWithMethod('DELETE')).toHaveLength(0);

    // The server created the session anyway and reports it with an absolute
    // Location — which is exactly what must be deleted. (It lies under the
    // endpoint's <path>/whep/, as the fetch-policy patch requires.)
    postDeferred.resolve(postResponse({ location: 'http://jetson.test:8889/tracking/whep/xyz' }));
    await flush();

    expect(callsWithMethod('DELETE')).toEqual([{
      url: 'http://jetson.test:8889/tracking/whep/xyz',
      method: 'DELETE',
      keepalive: true,
    }]);
    // The answer is never applied to a peer connection that is already closed.
    expect(readerPc().remoteDescriptions).toBe(0);

    await expectNoReconnect();
    // A close is not an operator-visible failure.
    expect(onError).not.toHaveBeenCalled();
  });

  it('cancels the retry the error path armed', async () => {
    const onError = vi.fn();
    const reader = new MediaMTXWebRTCReader({ url: WHEP_URL, onError });

    await flush();
    postDeferred.resolve(postResponse({ status: 404 }));
    await flush();

    // Upstream behaviour: a failed offer schedules its own retry.
    expect(onError).toHaveBeenCalledTimes(1);
    expect(onError.mock.calls[0][0]).toContain('stream not found');

    reader.close();
    await flush();

    // No session had been created, so there is nothing to delete...
    expect(callsWithMethod('DELETE')).toHaveLength(0);
    // ...and the pending retry must not fire after close().
    await expectNoReconnect();
  });
});

// NavPy patch 6 (see ./NOTICE): the browser is held to the endpoint it was
// given. A refused redirect rejects the fetch (redirect: 'error'), which looks
// exactly like a network failure, so every rejection is terminal.
describe('vendored WHEP reader fetch policy', () => {
  // Every fetch the reader has made so far, after letting any armed restart run.
  const settleAndCount = async () => {
    vi.advanceTimersByTime(30_000);
    await flush();
    return fetchCalls.length;
  };

  const localCandidate = () => ({ candidate: 'candidate:1 1 udp 1 10.0.0.1 9 typ host', sdpMLineIndex: 0 });

  it('sends every request with redirect "error" and no credentials', async () => {
    const reader = new MediaMTXWebRTCReader({ url: WHEP_URL, onError: vi.fn() });
    await flush();
    postDeferred.resolve(postResponse());
    await flush();
    readerPc().onicecandidate({ candidate: localCandidate() });
    await flush();
    reader.close();
    await flush();

    expect(fetchCalls.map((c) => c.method)).toEqual(['OPTIONS', 'POST', 'PATCH', 'DELETE']);
    for (const init of fetchInits) {
      expect(init.redirect).toBe('error');
      expect(init.credentials).toBe('omit');
    }
  });

  it('a redirect on the endpoint ends in error with no further fetch', async () => {
    optionsImpl = () => Promise.reject(new TypeError('Failed to fetch'));
    const onError = vi.fn();
    new MediaMTXWebRTCReader({ url: WHEP_URL, onError });
    await flush();

    expect(onError).toHaveBeenCalledTimes(1);
    expect(fetchCalls.map((c) => c.method)).toEqual(['OPTIONS']);
    // Upstream would re-offer after its retry pause; the patch must not.
    expect(await settleAndCount()).toBe(1);
    expect(onError).toHaveBeenCalledTimes(1);
  });

  it('a redirect on the offer ends in error with no further fetch', async () => {
    const onError = vi.fn();
    new MediaMTXWebRTCReader({ url: WHEP_URL, onError });
    await flush();
    postDeferred.resolve(Promise.reject(new TypeError('Failed to fetch')));
    await flush();

    expect(onError).toHaveBeenCalledTimes(1);
    expect(readerPc().closed).toBe(true);
    expect(fetchCalls.map((c) => c.method)).toEqual(['OPTIONS', 'POST']);
    expect(await settleAndCount()).toBe(2);
  });

  const rejectsLocation = (label, location) => {
    it(`a ${label} session Location ends in error with no further fetch`, async () => {
      const onError = vi.fn();
      const reader = new MediaMTXWebRTCReader({ url: WHEP_URL, onError });
      await flush();
      // A candidate gathered before the answer is queued for the session URL.
      readerPc().onicecandidate({ candidate: localCandidate() });
      postDeferred.resolve(postResponse({ location }));
      await flush();

      expect(onError).toHaveBeenCalledTimes(1);
      expect(onError.mock.calls[0][0]).toContain('outside the endpoint');
      expect(readerPc().closed).toBe(true);
      // Never PATCHed, never DELETEd: the foreign URL is not fetched at all.
      expect(fetchCalls.map((c) => c.method)).toEqual(['OPTIONS', 'POST']);
      expect(await settleAndCount()).toBe(2);

      // close() afterwards has nothing to release either.
      reader.close();
      await flush();
      expect(fetchCalls).toHaveLength(2);
    });
  };

  rejectsLocation('foreign-origin', 'http://evil.test:8889/tracking/whep/abc');
  rejectsLocation('other-port', 'http://jetson.test:9999/tracking/whep/abc');
  rejectsLocation('other-path', '/other/whep/abc');
  rejectsLocation('dot-dot escaping', '/tracking/whep/../../api/abc');
  rejectsLocation('bare endpoint', '/tracking/whep/');
  rejectsLocation('missing', null);

  it('does not DELETE a foreign Location that arrives after close()', async () => {
    const onError = vi.fn();
    const reader = new MediaMTXWebRTCReader({ url: WHEP_URL, onError });
    await flush();
    reader.close();
    postDeferred.resolve(postResponse({ location: 'http://evil.test/tracking/whep/abc' }));
    await flush();

    expect(callsWithMethod('DELETE')).toHaveLength(0);
    expect(onError).not.toHaveBeenCalled();
    expect(await settleAndCount()).toBe(2);
  });
});
