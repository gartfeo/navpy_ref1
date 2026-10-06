export function decodeBitmask(val) {
  const n = parseInt(val) || 0;
  const indices = [];
  for (let i = 0; i < 24; i++) { if (n & (1 << i)) indices.push(i + 1); }
  return indices.join(',');
}

export function decodeBitmaskArray(val) {
  const n = parseInt(val) || 0;
  const indices = [];
  for (let i = 0; i < 24; i++) { if (n & (1 << i)) indices.push(i + 1); }
  return indices;
}

export function encodeBitmask(text) {
  const indices = text.split(',').map((s) => parseInt(s.trim())).filter((n) => !isNaN(n) && n >= 1 && n <= 24);
  return indices.reduce((mask, i) => mask | (1 << (i - 1)), 0);
}
