import { useEffect, useState } from 'react';

/**
 * Measured size of an element (or of what `pick` returns for it), kept current
 * with a ResizeObserver. Until measured (or where there is no layout engine)
 * it reports `fallback`.
 */
export default function useElementSize(ref, fallback, pick = (el) => el) {
  const [size, setSize] = useState(fallback);
  useEffect(() => {
    const el = ref.current ? pick(ref.current) : null;
    if (!el || typeof ResizeObserver === 'undefined') return undefined;
    const measure = () => {
      const width = el.clientWidth;
      const height = el.clientHeight;
      if (width > 0 && height > 0) {
        setSize((s) => (s.width === width && s.height === height ? s : { width, height }));
      }
    };
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(el);
    return () => observer.disconnect();
  }, [ref]);
  return size;
}
