import { describe, it, expect, vi } from 'vitest';
import { render, fireEvent } from '@testing-library/react';
import VehicleStatusCard from './VehicleStatusCard';

const UAV = { sys_id: 3, name: 'UAV 3', link_ok: true, mode: 'FBWA', armed: false };

function card(props) {
  return render(
    <VehicleStatusCard
      vehicle={UAV}
      index={0}
      wpOffset={0}
      navpyStatus={{}}
      navpyBusy={{}}
      loadStage="ready"
      {...props}
    />,
  );
}

describe('VehicleStatusCard Messages button', () => {
  it('opens this UAV\'s Messages window', () => {
    const onOpenMessages = vi.fn();
    const view = card({ onOpenMessages });
    fireEvent.click(view.getByRole('button', { name: 'messages.title' }));
    expect(onOpenMessages).toHaveBeenCalledWith(3);
  });

  it('is absent without a handler', () => {
    const view = card({});
    expect(view.queryByRole('button', { name: 'messages.title' })).toBeNull();
  });
});
