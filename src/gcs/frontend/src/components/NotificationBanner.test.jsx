import { describe, it, expect } from 'vitest';
import { render } from '@testing-library/react';
import NotificationBanner from './NotificationBanner';

describe('NotificationBanner', () => {
  const banner = (props) => render(<NotificationBanner text="note" {...props} />).getByText('note');

  it('renders nothing without text', () => {
    const { container } = render(<NotificationBanner text={null} top={94} />);
    expect(container).toBeEmptyDOMElement();
  });

  it('sits at the bottom by default', () => {
    const { style } = banner();
    expect(style.bottom).toBe('60px');
    expect(style.top).toBe('');
  });

  // The collapsed confirm list can reach the bottom: while it shows, the toast moves up,
  // clear of a card's Approve and Deny (App passes the top).
  it('with a top it sits there instead', () => {
    const { style } = banner({ top: 94 });
    expect(style.top).toBe('94px');
    expect(style.bottom).toBe('');
  });

  // jsdom has no layout: these are the rules that make a long toast wrap inside a
  // 375 px pane, and let taps through to what lies under it. The live check measures them.
  it('stays inside a narrow pane and never takes a tap', () => {
    const { style } = banner();
    expect(style.width).toBe('max-content');
    expect(style.maxWidth).toBe('calc(100% - 24px)');
    expect(style.whiteSpace).toBe('');
    expect(style.pointerEvents).toBe('none');
  });
});
