// The app's 32 px map button: its default look is unchanged by the joined and
// unavailable variants the vehicle window uses (gcs_video_streams.md, v12.1).
import { describe, it, expect, vi } from 'vitest';
import { render, fireEvent } from '@testing-library/react';
import MapIconBtn from './MapIconBtn';

const renderBtn = (props = {}) => render(<MapIconBtn aria-label="b" {...props}>x</MapIconBtn>).container.querySelector('button');

describe('MapIconBtn', () => {
  it('default: 32 px, its own border and 6 px radius, no inset ring', () => {
    const b = renderBtn();
    expect(b.style.width).toBe('32px');
    expect(b.style.height).toBe('32px');
    expect(b.style.borderWidth).toBe('1px');
    expect(b.style.borderStyle).toBe('solid');
    expect(b.style.borderRadius).toBe('6px');
    expect(b.style.boxShadow).toBe('');
    expect(b.style.cursor).toBe('pointer');
    expect(b.style.opacity).toBe('1');
  });

  it('active: the accent border, as before', () => {
    const plain = renderBtn();
    const active = render(<MapIconBtn aria-label="a" active>x</MapIconBtn>).container.querySelector('button');
    expect(active.style.borderColor).not.toBe(plain.style.borderColor);
    expect(active.style.boxShadow).toBe('');
  });

  it('joined: no border or radius of its own, a left divider, the pressed ring inside', () => {
    const b = render(<MapIconBtn aria-label="j" joined active>x</MapIconBtn>).container.querySelector('button');
    expect(b.style.borderTopStyle).toBe('none');
    expect(b.style.borderLeftWidth).toBe('1px');
    expect(b.style.borderLeftStyle).toBe('solid');
    expect(b.style.borderRadius).toBe('0px');
    expect(b.style.boxShadow).toContain('inset');
  });

  it('unavailable: dimmed but enabled, focusable and clickable', () => {
    const onClick = vi.fn();
    const b = renderBtn({ unavailable: true, onClick });
    expect(b.disabled).toBe(false);
    expect(b.style.opacity).toBe('0.7');
    expect(b.style.cursor).toBe('not-allowed');
    b.focus();
    expect(document.activeElement).toBe(b);
    fireEvent.click(b);
    expect(onClick).toHaveBeenCalledTimes(1);
  });
});
