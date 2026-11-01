import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, act, fireEvent, within } from '@testing-library/react';
import TaskConfirmCard, { ConfirmStill } from './TaskConfirmCard';
import ConfirmCardList from './ConfirmCardList';
import useTaskConfirmation from '../hooks/useTaskConfirmation';
import { colors } from '../styles';

// Todo 71 (gcs_still_age.md): the still's age and POI. The caption's words are
// the point here, so this file resolves the real English strings instead of
// the setup file's key-only stub.
vi.mock('react-i18next', async () => {
  const en = (await import('../locales/en.json')).default;
  const lookup = (key) => key.split('.').reduce((node, part) => node?.[part], en);
  return {
    useTranslation: () => ({
      t: (key, options = {}) => (lookup(key) ?? key).replace(/{{\s*(\w+)\s*}}/g, (_, name) => options[name]),
      i18n: { language: 'en', changeLanguage: () => Promise.resolve() },
    }),
    initReactI18next: { type: '3rdParty', init: () => {} },
  };
});

const t0 = 1_000_000;

beforeEach(() => {
  vi.useFakeTimers({ toFake: ['Date', 'setInterval', 'clearInterval'] });
  vi.setSystemTime(t0);
});

const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });

const WAIT_SEC = 30;
const IMAGE = 'aGVsbG8=';
const NO_OP = () => {};

function entryAt(receivedAt, overrides = {}) {
  return {
    taskId: 7,
    roundUid: 'r1',
    taskType: 'DOCK',
    lat: 32.5,
    lon: 34.8,
    alt: 100,
    imageB64: null,
    imageAt: null,
    status: 'pending',
    action: null,
    receivedAt,
    decidedAt: null,
    ...overrides,
  };
}

function hostedCard(entry, props = {}) {
  return (
    <TaskConfirmCard
      sysId={1} entry={entry} vehicleName="UAV 1" vehicleIndex={0}
      onApprove={NO_OP} onDeny={NO_OP} onCancel={NO_OP}
      autoApprove={false} timeoutSec={WAIT_SEC} labelPolicy={{ cmFl: 0, cwt: WAIT_SEC }}
      {...props}
    />
  );
}

/** The app's list with the real hook, which stamps the receipt times. */
function setupList() {
  const control = { current: null };
  function Harness() {
    const tc = useTaskConfirmation();
    control.current = tc;
    return (
      <ConfirmCardList
        pendingConfirms={tc.pendingConfirms} vehicleList={[{ sys_id: 1, name: 'UAV 1' }]}
        onApprove={tc.approve} onDeny={tc.deny} onCancel={tc.cancel}
        forcedConfirms={tc.forcedConfirms} cue={tc.cue}
        aasParams={{ getConfirmedValue: (sid, key, fallback) => (key === 'nav_cwt' ? WAIT_SEC : fallback) }}
      />
    );
  }
  return { ...render(<Harness />), control };
}

const request = (control, { taskId = 7, roundUid = 'r1' } = {}) => act(() => {
  control.current.handleConfirmRequest({ sys_id: 1, task_id: taskId, round_uid: roundUid, lat: 32.5, lon: 34.8, alt: 100 });
});
const image = (control, { taskId = 7, roundUid = 'r1', data = IMAGE } = {}) => act(() => {
  control.current.handleConfirmImage({ sys_id: 1, task_id: taskId, round_uid: roundUid, image_b64: data });
});

describe('pending card: the caption', () => {
  it('shows the empty box and the age before the image, with no capture wording', () => {
    const { control, getByText, getByTestId, queryByRole, container } = setupList();
    request(control);
    expect(getByText('Waiting for image')).toBeInTheDocument();
    expect(queryByRole('img')).toBeNull();
    expect(getByTestId('still-caption')).toHaveTextContent('POI #7 · request received 0:00 ago');
    expect(container.textContent).not.toMatch(/captur/i);
  });

  it('keeps the same caption, adds the note and records the delay when the image arrives', () => {
    const { control, getByTestId, getByRole, queryByText, container } = setupList();
    request(control);
    advance(3000);
    image(control);
    expect(getByRole('img')).toHaveAttribute('src', `data:image/jpeg;base64,${IMAGE}`);
    expect(queryByText('Waiting for image')).toBeNull();
    expect(getByTestId('still-caption')).toHaveTextContent('POI #7 · request received 0:03 ago');
    expect(getByTestId('still-age-note')).toHaveTextContent('Taken before the request; exact time unknown');
    expect(getByTestId('still-age-note').style.visibility).toBe('visible');
    expect(getByTestId('confirm-picture-1').dataset.imageDelayMs).toBe('3000');
    expect(container.textContent).not.toMatch(/captur/i);
  });

  it('advances the age with the clock', () => {
    const { control, getByTestId } = setupList();
    request(control);
    image(control);
    advance(7000);
    expect(getByTestId('still-caption')).toHaveTextContent('request received 0:07 ago');
    advance(60_000);
    expect(getByTestId('still-caption')).toHaveTextContent('request received 1:07 ago');
  });

  it('names the POI of this round, as the header does', () => {
    const { getByTestId, getByText } = render(hostedCard(entryAt(Date.now(), { taskId: 42 }), { now: Date.now() }));
    expect(getByText('POI #42')).toBeInTheDocument();
    expect(getByTestId('still-caption')).toHaveTextContent('POI #42 ·');
  });

  it('draws the age line brighter than the note under it', () => {
    const { getByTestId } = render(hostedCard(entryAt(t0, { imageB64: IMAGE }), { now: t0 }));
    expect(getByTestId('still-caption')).toHaveStyle({ color: colors.text });
    expect(getByTestId('still-age-note')).toHaveStyle({ color: colors.textDim });
  });
});

describe('pending card: the empty box by phase', () => {
  it('says the image is not in once the countdown reaches zero, and at fired', () => {
    const { getByText, queryByText, rerender } = render(hostedCard(entryAt(t0), { now: t0 + 29_000 }));
    expect(getByText('Waiting for image')).toBeInTheDocument();
    rerender(hostedCard(entryAt(t0), { now: t0 + WAIT_SEC * 1000 }));
    expect(getByText('No image received')).toBeInTheDocument();
    rerender(hostedCard(entryAt(t0), { now: t0 + 29_000, fired: true }));
    expect(getByText('No image received')).toBeInTheDocument();
    expect(queryByText('Waiting for image')).toBeNull();
  });

  it('a zero wait has no live period: no image received from the start', () => {
    const { getByText } = render(hostedCard(entryAt(t0), { now: t0, timeoutSec: 0, labelPolicy: { cmFl: 0, cwt: 0 } }));
    expect(getByText('No image received')).toBeInTheDocument();
  });

  it('a late image replaces the words', () => {
    const { getByRole, queryByText, rerender } = render(hostedCard(entryAt(t0), { now: t0 + 40_000, fired: true }));
    rerender(hostedCard(entryAt(t0, { imageB64: IMAGE, imageAt: t0 + 40_000 }), { now: t0 + 40_000, fired: true }));
    expect(queryByText('No image received')).toBeNull();
    expect(getByRole('img')).toBeInTheDocument();
  });
});

describe('pending card: the age and the countdown', () => {
  const shown = (view) => ({
    age: view.getByTestId('still-caption').dataset.age,
    countdown: view.getByTestId('confirm-countdown-1').dataset.time,
  });
  const seconds = (clock) => {
    const [m, s] = clock.split(':').map(Number);
    return m * 60 + s;
  };

  it('add up to the wait at every offset while the round is unfired', () => {
    const onDeny = vi.fn();
    for (const offset of [0, 1, 999, 1000, 1001, 1500, 7250, 29_000, 29_999]) {
      const view = render(hostedCard(entryAt(t0), { now: t0 + offset, onDeny }));
      const { age, countdown } = shown(view);
      expect(seconds(age) + seconds(countdown), `offset ${offset}`).toBe(WAIT_SEC);
      view.unmount();
    }
    expect(onDeny).not.toHaveBeenCalled();
  });

  it('after expiry the countdown holds at zero and the age keeps rising', () => {
    const view = render(hostedCard(entryAt(t0), { now: t0 + 45_000, fired: true }));
    expect(shown(view)).toEqual({ age: '0:45', countdown: '0:00' });
  });

  it('follows a clock step, and never reads negative', () => {
    const view = render(hostedCard(entryAt(t0), { now: t0 + 10_000 }));
    expect(shown(view).age).toBe('0:10');
    view.rerender(hostedCard(entryAt(t0), { now: t0 + 7000 }));
    expect(shown(view).age).toBe('0:07');
    view.rerender(hostedCard(entryAt(t0), { now: t0 - 5000 }));
    expect(shown(view).age).toBe('0:00');
    view.rerender(hostedCard(entryAt(t0), { now: t0 + 100_000 }));
    expect(shown(view).age).toBe('1:40');
  });

  it('a late NAV_CWT load moves the countdown, not the age', () => {
    const notLoaded = { timeoutSec: undefined, labelPolicy: { cmFl: undefined, cwt: undefined } };
    const view = render(hostedCard(entryAt(t0), { now: t0 + 10_000, ...notLoaded }));
    expect(shown(view)).toEqual({ age: '0:10', countdown: '~0:20' });
    view.rerender(hostedCard(entryAt(t0), { now: t0 + 10_000, timeoutSec: 60, labelPolicy: { cmFl: 0, cwt: 60 } }));
    expect(shown(view)).toEqual({ age: '0:10', countdown: '0:50' });
  });

  it('a card alone reads the clock itself until it fires, then stops ticking', () => {
    const view = render(hostedCard(entryAt(Date.now()), { labelPolicy: null }));
    expect(view.getByTestId('still-caption').dataset.age).toBe('0:00');
    advance(7000);
    expect(view.getByTestId('still-caption').dataset.age).toBe('0:07');
    advance(23_000);
    expect(view.getByTestId('still-caption').dataset.age).toBe('0:30');
    advance(5000);
    expect(view.getByTestId('still-caption').dataset.age).toBe('0:30');
  });
});

describe('pending card: rounds', () => {
  it('a re-ask restarts the age and clears the image', () => {
    const { control, getByTestId, queryByRole, getByText } = setupList();
    request(control, { roundUid: 'r1' });
    image(control, { roundUid: 'r1' });
    advance(9000);
    expect(getByTestId('still-caption')).toHaveTextContent('0:09 ago');
    request(control, { roundUid: 'r2' });
    expect(getByTestId('still-caption')).toHaveTextContent('0:00 ago');
    expect(queryByRole('img')).toBeNull();
    expect(getByText('Waiting for image')).toBeInTheDocument();
    expect(getByTestId('confirm-picture-1').dataset.imageDelayMs).toBeUndefined();
  });

  it('an image with another round uid neither attaches nor changes the caption', () => {
    const { control, getByTestId, queryByRole } = setupList();
    request(control, { roundUid: 'r2' });
    advance(4000);
    const before = getByTestId('still-caption').textContent;
    image(control, { roundUid: 'r1' });
    expect(queryByRole('img')).toBeNull();
    expect(getByTestId('still-caption').textContent).toBe(before);
    expect(getByTestId('still-age-note').style.visibility).toBe('hidden');
    expect(getByTestId('confirm-picture-1').dataset.imageDelayMs).toBeUndefined();
  });

  it('an image with no round uid falls back to the task id', () => {
    const { control, getByRole, queryByRole } = setupList();
    request(control, { taskId: 7, roundUid: 'r2' });
    image(control, { taskId: 8, roundUid: null });
    expect(queryByRole('img')).toBeNull();
    image(control, { taskId: 7, roundUid: null });
    expect(getByRole('img')).toBeInTheDocument();
  });

  it('a request with no round uid takes an image that has one, without moving the age', () => {
    const { control, getByRole, getByTestId } = setupList();
    request(control, { roundUid: null });
    advance(2000);
    image(control, { roundUid: 'r9' });
    expect(getByRole('img')).toBeInTheDocument();
    expect(getByTestId('confirm-picture-1').dataset.imageDelayMs).toBe('2000');
    expect(getByTestId('still-caption').dataset.age).toBe('0:02');
  });
});

describe('pending card: nothing moves when the image arrives (decision 6)', () => {
  it('the picture has one height with and without the image', () => {
    const empty = render(hostedCard(entryAt(t0), { now: t0 }));
    const boxHeight = empty.getByText('Waiting for image').style.height;
    empty.unmount();
    const full = render(hostedCard(entryAt(t0, { imageB64: IMAGE, imageAt: t0 }), { now: t0 }));
    const img = full.getByRole('img');
    expect(boxHeight).toBe('160px');
    expect(img.style.height).toBe(boxHeight);
    expect(img.style.objectFit).toBe('contain');
    expect(img.style.maxHeight).toBe('');
  });

  it('both caption lines are rendered in both states; the note is only hidden before the image', () => {
    const view = render(hostedCard(entryAt(t0), { now: t0 }));
    expect(view.getByTestId('still-caption').style.visibility).toBe('');
    expect(view.getByTestId('still-age-note')).toHaveTextContent('Taken before the request');
    expect(view.getByTestId('still-age-note').style.visibility).toBe('hidden');
    view.rerender(hostedCard(entryAt(t0, { imageB64: IMAGE, imageAt: t0 }), { now: t0 }));
    expect(view.getByTestId('still-age-note').style.visibility).toBe('visible');
  });
});

describe('pending card: View camera note', () => {
  const videoAction = { viewing: false, onView: NO_OP };
  const countdownSentence = (view) => view.getByText('The countdown keeps running.');

  it('names the POI and keeps the countdown sentence while the round is live', () => {
    const view = render(hostedCard(entryAt(t0), { now: t0, videoAction }));
    expect(view.getByText(/Live view may no longer show POI #7\./)).toBeInTheDocument();
    expect(countdownSentence(view).style.visibility).toBe('visible');
  });

  it('hides the countdown sentence, keeping its space, once due, fired or with no wait', () => {
    const view = render(hostedCard(entryAt(t0), { now: t0 + WAIT_SEC * 1000, videoAction }));
    expect(countdownSentence(view).style.visibility).toBe('hidden');
    view.rerender(hostedCard(entryAt(t0), { now: t0 + 1000, fired: true, videoAction }));
    expect(countdownSentence(view).style.visibility).toBe('hidden');
    view.rerender(hostedCard(entryAt(t0), { now: t0, timeoutSec: 0, labelPolicy: { cmFl: 0, cwt: 0 }, videoAction }));
    expect(countdownSentence(view).style.visibility).toBe('hidden');
    expect(view.getByText(/Live view may no longer show POI #7\./)).toBeInTheDocument();
  });
});

describe('opened still of an approved round', () => {
  const stillOf = (entry) => <ConfirmStill entry={entry} vehicleName="UAV 1" vehicleIndex={0} />;

  it('shows the caption and ticks on its own while open', () => {
    const entry = entryAt(Date.now(), { status: 'approved', imageB64: IMAGE, imageAt: Date.now() });
    const view = render(stillOf(entry));
    expect(view.getByTestId('still-caption')).toHaveTextContent('POI #7 · request received 0:00 ago');
    expect(view.getByTestId('still-age-note')).toHaveTextContent('Taken before the request');
    expect(view.getByTestId('still-age-note').style.visibility).toBe('visible');
    advance(12_000);
    expect(view.getByTestId('still-caption')).toHaveTextContent('request received 0:12 ago');
    expect(view.container.textContent).not.toMatch(/captur/i);
  });

  it('runs one timer while open and none once closed', () => {
    const entry = entryAt(Date.now(), { status: 'approved', imageB64: IMAGE, imageAt: Date.now() });
    expect(vi.getTimerCount()).toBe(0);
    const view = render(stillOf(entry));
    expect(vi.getTimerCount()).toBe(1);
    view.unmount();
    expect(vi.getTimerCount()).toBe(0);
  });

  it('keeps ticking in the list when no round is pending', () => {
    vi.spyOn(globalThis, 'fetch').mockResolvedValue({ ok: true, status: 200 });
    const { control, getByTitle, getByTestId } = setupList();
    request(control);
    image(control);
    act(() => {
      control.current.handleConfirmResponse({ sys_id: 1, task_id: 7, round_uid: 'r1', is_confirmed: true, action: 'approve' });
    });
    fireEvent.click(getByTitle('Show still'));
    const still = within(getByTestId('confirm-still'));
    expect(still.getByTestId('still-caption')).toHaveTextContent('0:00 ago');
    advance(5000);
    expect(still.getByTestId('still-caption')).toHaveTextContent('0:05 ago');
  });
});
