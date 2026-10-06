/**
 * Fetch wrapper with exponential-backoff retry for network and server errors.
 *
 * Retries on: network exceptions (TypeError), 5xx, 408 (Request Timeout).
 * Does NOT retry on 4xx client errors (except 408).
 */
export default async function fetchWithRetry(url, options = {}, { maxRetries = 2, baseDelay = 500 } = {}) {
  for (let attempt = 0; ; attempt++) {
    try {
      const res = await fetch(url, options);
      if (res.ok || attempt >= maxRetries) return res;
      // Retry on server errors and 408
      if (res.status >= 500 || res.status === 408) {
        await new Promise((r) => setTimeout(r, baseDelay * 2 ** attempt));
        continue;
      }
      return res; // 4xx — don't retry
    } catch (err) {
      // An aborted request (caller timeout) is terminal — retrying would just
      // hit the already-fired deadline again and mask the timeout as a network
      // error. Propagate it so callers can surface a bounded "timeout".
      if (err.name === 'AbortError' || attempt >= maxRetries) throw err;
      await new Promise((r) => setTimeout(r, baseDelay * 2 ** attempt));
    }
  }
}
