import { describe, it, expect, vi } from 'vitest';
import { render } from '@testing-library/react';
import ManualControlOverlay, { STICK_INSET_PX, STICK_SIZE_PX } from './ManualControlOverlay';
import VehicleSelector, { VEHICLE_SELECTOR_SPAN_PX } from './VehicleSelector';
import FpvViewOverlay, { FPV_TAG_HEIGHT_PX } from './map/FpvViewOverlay';
import FlightModeColumn, { MODE_COLUMN_INSET_PX, MODE_BUTTON_WIDTH_PX } from './FlightModeColumn';

// App places the RC confirm list (todo 69, decision 4c) from these sizes:
// right edge 8 px left of the right stick, bottom 8 px above the UAV
// selector, top 8 px below the FPV tag; while it shows, the "View held"
// banner starts 8 px right of the flight-mode buttons. They must be the sizes
// the components really draw, or the list or banner lands on a control.

// jsdom has no canvas: stand in for the stick with its size.
vi.mock('./VirtualJoystick', () => ({
  default: ({ size }) => <div data-testid="stick" data-size={size} />,
}));

describe('RC confirm list geometry sources', () => {
  it('the sticks are STICK_SIZE_PX square at STICK_INSET_PX from the bottom corners', () => {
    const { container } = render(<ManualControlOverlay onLeftMove={() => {}} onRightMove={() => {}} />);
    const [left, right] = container.children;
    expect(left.style.bottom).toBe(`${STICK_INSET_PX}px`);
    expect(right.style.bottom).toBe(`${STICK_INSET_PX}px`);
    expect(right.style.right).toBe(`${STICK_INSET_PX}px`);
    expect(right.querySelector('[data-testid="stick"]').dataset.size).toBe(String(STICK_SIZE_PX));
    expect(STICK_INSET_PX + STICK_SIZE_PX).toBe(174);
  });

  it('the selector spans VEHICLE_SELECTOR_SPAN_PX above the map bottom with its "Select UAV" line', () => {
    const fleet = [{ sys_id: 7, name: 's1-u7' }];
    const { getByText, getByRole } = render(<VehicleSelector vehicleList={fleet} selectedSysId={null} onSelect={() => {}} />);
    const box = getByText('vehicleSelector.selectUav').parentElement;
    const label = parseFloat(getByText('vehicleSelector.selectUav').style.lineHeight);
    const chip = parseFloat(getByRole('button').style.height);
    expect(parseFloat(box.style.bottom) + label + parseFloat(box.style.gap) + chip).toBe(VEHICLE_SELECTOR_SPAN_PX);
    expect(VEHICLE_SELECTOR_SPAN_PX).toBe(77);
  });

  it('the flight-mode buttons are MODE_BUTTON_WIDTH_PX wide at MODE_COLUMN_INSET_PX from the top-left corner', () => {
    const { container, getAllByRole } = render(
      <FlightModeColumn currentMode="FBWA" armed={false} sysId={7} onSetMode={() => {}} onArmDisarm={() => {}} onToggleManualControl={() => {}} />,
    );
    const column = container.firstChild.style;
    expect(column.left).toBe(`${MODE_COLUMN_INSET_PX}px`);
    expect(column.top).toBe(`${MODE_COLUMN_INSET_PX}px`);
    for (const button of getAllByRole('button')) expect(button.style.width).toBe(`${MODE_BUTTON_WIDTH_PX}px`);
    expect(MODE_COLUMN_INSET_PX + MODE_BUTTON_WIDTH_PX).toBe(92);
  });

  it('the FPV tag is FPV_TAG_HEIGHT_PX tall', () => {
    const { getByRole } = render(<FpvViewOverlay label="UAV 7" holdReason={null} top={54} />);
    const tag = getByRole('note').style;
    expect(parseFloat(tag.lineHeight) + 2 * parseFloat(tag.paddingTop) + 2).toBe(FPV_TAG_HEIGHT_PX);
    expect(tag.border).toContain('1px');
  });
});
