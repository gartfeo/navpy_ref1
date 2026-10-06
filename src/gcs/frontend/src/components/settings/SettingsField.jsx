import React from 'react';
import { colors, zoneColorsLabel } from '../../styles';

export const inputStyle = {
  flex: 1,
  maxWidth: 120,
  background: colors.bg,
  border: `1px solid ${colors.border}`,
  borderRadius: 4,
  padding: '4px 8px',
  color: colors.textBright,
  fontSize: 13,
};

export const changedStyle = {
  borderLeft: `3px solid ${colors.accent}`,
  paddingLeft: 6,
};

export function DiffsRow({ diffs, indent = 208 }) {
  if (!diffs || diffs.length === 0) return null;
  return (
    <div style={{ display: 'flex', gap: 10, overflowX: 'auto', marginLeft: indent, fontSize: 11, color: colors.textDim, whiteSpace: 'nowrap' }}>
      {diffs.map((d) => (
        <span key={d.label}>
          <span style={{ color: zoneColorsLabel[d.colorIdx % zoneColorsLabel.length] }}>{d.label}</span>
          : <span style={{ color: colors.text }}>{String(d.value)}</span>
        </span>
      ))}
    </div>
  );
}

/**
 * Controlled numeric input that allows intermediate typing states
 * (e.g. "-", ".", "-.") without clobbering the display.
 */
export function NumericInput({ value, onChange, step, min, max, style }) {
  const [text, setText] = React.useState(value != null ? String(value) : '');
  const internalRef = React.useRef(value);

  React.useEffect(() => {
    if (value !== internalRef.current) {
      internalRef.current = value;
      setText(value != null ? String(value) : '');
    }
  }, [value]);

  const clamp = (n) => {
    if (min != null && n < min) return min;
    if (max != null && n > max) return max;
    return n;
  };

  const push = (n) => {
    internalRef.current = n;
    onChange(n);
  };

  return (
    <input
      type="text"
      inputMode="decimal"
      value={text}
      onChange={(e) => {
        const raw = e.target.value;
        setText(raw);
        if (raw === '' || raw === '-' || raw === '.' || raw === '-.') return;
        const n = parseFloat(raw);
        if (!isNaN(n)) push(clamp(n));
      }}
      onBlur={() => {
        const n = parseFloat(text);
        if (!isNaN(n)) {
          const c = clamp(n);
          setText(String(c));
          push(c);
        } else {
          setText(value != null ? String(value) : '');
        }
      }}
      onKeyDown={(e) => {
        if (e.key !== 'ArrowUp' && e.key !== 'ArrowDown') return;
        e.preventDefault();
        const cur = parseFloat(text) || 0;
        const s = step || 1;
        const next = clamp(e.key === 'ArrowUp' ? cur + s : cur - s);
        const dec = String(s).includes('.') ? String(s).split('.')[1].length : 0;
        const rounded = parseFloat(next.toFixed(dec));
        setText(String(rounded));
        push(rounded);
      }}
      style={style || inputStyle}
    />
  );
}

/**
 * Small "i" info badge. Shows `text` as a native tooltip on hover.
 */
export function InfoIcon({ text }) {
  if (!text) return null;
  return (
    <span
      title={text}
      aria-label={text}
      style={{
        display: 'inline-flex',
        alignItems: 'center',
        justifyContent: 'center',
        width: 13,
        height: 13,
        marginLeft: 5,
        borderRadius: '50%',
        border: `1px solid ${colors.textDim}`,
        color: colors.textDim,
        fontSize: 9,
        fontWeight: 700,
        fontStyle: 'normal',
        lineHeight: 1,
        cursor: 'help',
        flexShrink: 0,
        verticalAlign: 'middle',
      }}
    >
      i
    </span>
  );
}

export function SettingsField({ label, value, onChange, type = 'number', step, description, options, min, max, isChanged, tooltip, diffs, info }) {
  const rowExtra = isChanged ? changedStyle : {};
  const tipProps = tooltip ? { title: tooltip } : {};
  if (type === 'checkbox') {
    return (
      <div>
        <div style={{ display: 'flex', alignItems: 'center', gap: 9, fontSize: 13, ...rowExtra }}>
          <label style={{ color: colors.text, minWidth: 200, flexShrink: 0, cursor: 'pointer' }} {...tipProps}>
            {label}<InfoIcon text={info} />
            {description && <div style={{ color: colors.textDim, fontSize: 11 }}>{description}</div>}
          </label>
          <input
            type="checkbox"
            checked={!!value}
            onChange={(e) => onChange(e.target.checked)}
            style={{ accentColor: colors.accent, cursor: 'pointer' }}
          />
        </div>
        <DiffsRow diffs={diffs} />
      </div>
    );
  }
  if (type === 'select' && options) {
    return (
      <div>
        <div style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 13, ...rowExtra }}>
          <label style={{ color: colors.text, minWidth: 200, flexShrink: 0 }} {...tipProps}>
            {label}<InfoIcon text={info} />
            {description && <div style={{ color: colors.textDim, fontSize: 11 }}>{description}</div>}
          </label>
          <select
            value={value ?? ''}
            onChange={(e) => onChange(options.find((o) => String(o.value) === e.target.value)?.value ?? value)}
            style={{ ...inputStyle, cursor: 'pointer' }}
          >
            {options.map((o) => (
              <option key={o.value} value={o.value}>{o.label}</option>
            ))}
          </select>
        </div>
        <DiffsRow diffs={diffs} />
      </div>
    );
  }
  if (type === 'number') {
    return (
      <div>
        <div style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 13, ...rowExtra }}>
          <label style={{ color: colors.text, minWidth: 200, flexShrink: 0 }} {...tipProps}>
            {label}<InfoIcon text={info} />
            {description && <div style={{ color: colors.textDim, fontSize: 11 }}>{description}</div>}
          </label>
          <NumericInput value={value} onChange={onChange} step={step} min={min} max={max} />
        </div>
        <DiffsRow diffs={diffs} />
      </div>
    );
  }
  return (
    <div>
      <div style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 13, ...rowExtra }}>
        <label style={{ color: colors.text, minWidth: 200, flexShrink: 0 }} {...tipProps}>
          {label}<InfoIcon text={info} />
          {description && <div style={{ color: colors.textDim, fontSize: 11 }}>{description}</div>}
        </label>
        <input
          type={type}
          value={value ?? ''}
          onChange={(e) => onChange(e.target.value)}
          style={inputStyle}
        />
      </div>
      <DiffsRow diffs={diffs} />
    </div>
  );
}

export function SettingsSection({ title, children }) {
  return (
    <div style={{ marginBottom: 16 }}>
      <div style={{ fontSize: 13, fontWeight: 600, color: colors.accent, marginBottom: 8, textTransform: 'uppercase', letterSpacing: 1 }}>
        {title}
      </div>
      <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
        {children}
      </div>
    </div>
  );
}
