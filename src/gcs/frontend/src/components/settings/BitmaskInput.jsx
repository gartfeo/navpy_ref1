import React, { useState, useRef, useEffect } from 'react';
import { decodeBitmask, encodeBitmask } from '../../utils/bitmask.js';

export default function BitmaskInput({ value, onChange, style, placeholder }) {
  const [text, setText] = useState(() => decodeBitmask(value));
  const focusedRef = useRef(false);

  useEffect(() => {
    if (!focusedRef.current) setText(decodeBitmask(value));
  }, [value]);

  return (
    <input
      type="text"
      value={text}
      placeholder={placeholder}
      style={style}
      onFocus={() => { focusedRef.current = true; }}
      onBlur={() => {
        focusedRef.current = false;
        const mask = encodeBitmask(text);
        onChange(mask);
        setText(decodeBitmask(mask));
      }}
      onChange={(e) => setText(e.target.value)}
    />
  );
}
