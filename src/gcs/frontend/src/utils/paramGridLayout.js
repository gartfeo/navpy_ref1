// Column layout math for VirtualParamGrid. Pure + tested so the sticky header
// and the scrolling body share ONE source of truth for column widths and the
// total grid width. Adding the compare File column (and, in step 5, the
// selection checkbox) can't make the header and body diverge under horizontal
// scroll, because both derive every width from here.
//
// Column order: [checkbox?] [Name (sticky-left)] [File?] [UAV…]
//   - checkbox: in compare/harmonize modes when selectable.
//   - File:     only in compare mode (harmonize has no reference-file column).
// mode is one of 'normal' | 'compare' | 'harmonize'.
// Tested via tests/gcs/test_param_grid_layout_js.py (Node subprocess).

export const GRID_DIMS = {
  rowHeight: 28,
  headerHeight: 32,
  nameWidth: 220,
  valWidth: 124,
  fileWidth: 96,
  checkboxWidth: 34,
};

/**
 * Resolve the active column widths and total width.
 *
 *   vehicleCount  number of UAV value columns
 *   mode          'normal' | 'compare' | 'harmonize' — 'compare' adds the File column
 *   selectable    adds the leading checkbox column (compare/harmonize modes)
 *
 * fileWidth / checkboxWidth are 0 when their column is absent, so callers can
 * use them directly as the rendered width. totalWidth is the sum of every
 * present column — the explicit width the header row and the body spacer both use.
 */
export function computeGridLayout({
  vehicleCount = 0, mode = 'normal', selectable = false,
} = {}) {
  const n = Math.max(0, Math.floor(Number(vehicleCount) || 0));
  const isCompare = mode === 'compare';
  const isHarmonize = mode === 'harmonize';
  const checkboxWidth = (selectable && (isCompare || isHarmonize)) ? GRID_DIMS.checkboxWidth : 0;
  const fileWidth = isCompare ? GRID_DIMS.fileWidth : 0;
  const totalWidth = checkboxWidth + GRID_DIMS.nameWidth + fileWidth + n * GRID_DIMS.valWidth;
  return {
    rowHeight: GRID_DIMS.rowHeight,
    headerHeight: GRID_DIMS.headerHeight,
    nameWidth: GRID_DIMS.nameWidth,
    valWidth: GRID_DIMS.valWidth,
    fileWidth,
    checkboxWidth,
    vehicleCount: n,
    totalWidth,
  };
}
