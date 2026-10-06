import React, { useState, useEffect } from 'react';
import { useTranslation } from 'react-i18next';

const FADE_DURATION = 600;

// V-formation: lead at front, two pairs trailing back symmetrically
const FORMATION = [
  { dx: 0, dy: 0, size: 18 },      // lead
  { dx: -28, dy: -18, size: 15 },   // upper wing
  { dx: -28, dy: 18, size: 15 },    // lower wing
  { dx: -56, dy: -36, size: 13 },   // upper tail
  { dx: -56, dy: 36, size: 13 },    // lower tail
];

function FormationGroup({ delay, trailDelay }) {
  return (
    <>
      {/* Ghost trail */}
      <div style={{
        position: 'absolute',
        inset: 0,
        animation: `ls-fly 3.5s linear infinite`,
        animationDelay: `${trailDelay}s`,
      }}>
        {FORMATION.map((pos, i) => (
          <svg
            key={`t-${i}`}
            width={pos.size} height={pos.size} viewBox="0 0 16 16"
            style={{
              position: 'absolute',
              left: `calc(50% + ${pos.dx}px - ${pos.size / 2}px)`,
              top: `calc(50% + ${pos.dy}px - ${pos.size / 2}px)`,
              opacity: 0.12,
            }}
          >
            <polygon points="16,8 0,16 4,8 0,0" fill="#00d2ff" />
          </svg>
        ))}
      </div>
      {/* Main */}
      <div style={{
        position: 'absolute',
        inset: 0,
        animation: `ls-fly 3.5s linear infinite`,
        animationDelay: `${delay}s`,
      }}>
        {FORMATION.map((pos, i) => (
          <svg
            key={i}
            width={pos.size} height={pos.size} viewBox="0 0 16 16"
            style={{
              position: 'absolute',
              left: `calc(50% + ${pos.dx}px - ${pos.size / 2}px)`,
              top: `calc(50% + ${pos.dy}px - ${pos.size / 2}px)`,
              filter: 'drop-shadow(0 0 6px rgba(0,210,255,0.6))',
            }}
          >
            <polygon points="16,8 0,16 4,8 0,0" fill="#00d2ff" />
          </svg>
        ))}
      </div>
    </>
  );
}

/**
 * Full-screen loading overlay with AAS logo at dead center,
 * continuous delta formations above, and "Loading ..." below.
 */
export default function LoadingScreen({ visible }) {
  const { t } = useTranslation();
  const [mounted, setMounted] = useState(true);
  const [fading, setFading] = useState(false);

  useEffect(() => {
    if (visible) {
      setMounted(true);
      setFading(false);
    } else {
      setFading(true);
      const timer = setTimeout(() => setMounted(false), FADE_DURATION);
      return () => clearTimeout(timer);
    }
  }, [visible]);

  if (!mounted) return null;

  return (
    <div style={{
      position: 'fixed',
      inset: 0,
      zIndex: 9999,
      background: 'radial-gradient(ellipse at 50% 50%, #1f2b4d 0%, #1a1a2e 60%, #111122 100%)',
      opacity: fading ? 0 : 1,
      transition: `opacity ${FADE_DURATION}ms ease-out`,
      pointerEvents: fading ? 'none' : 'auto',
      overflow: 'hidden',
    }}>
      <style>{`
        @keyframes ls-fly {
          0%   { transform: translateX(-320px); opacity: 0; }
          8%   { opacity: 1; }
          92%  { opacity: 1; }
          100% { transform: translateX(320px); opacity: 0; }
        }
        @keyframes ls-glow {
          0%, 100% { opacity: 0.15; }
          50%      { opacity: 0.35; }
        }
      `}</style>

      {/* Logo — absolute dead center of the screen */}
      <div style={{
        position: 'absolute',
        top: '50%',
        left: '50%',
        transform: 'translate(-50%, -50%)',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
      }}>
        {/* Radial glow behind logo */}
        <div style={{
          position: 'absolute',
          width: 400,
          height: 400,
          top: '50%',
          left: '50%',
          transform: 'translate(-50%, -50%)',
          borderRadius: '50%',
          background: 'radial-gradient(circle, rgba(0,210,255,0.12) 0%, transparent 70%)',
          animation: 'ls-glow 3s ease-in-out infinite',
          pointerEvents: 'none',
        }} />
        <img
          src="/aas-logo.svg"
          alt="AAS"
          style={{
            width: 140,
            height: 140,
            filter: 'drop-shadow(0 0 24px rgba(0,210,255,0.3))',
            position: 'relative',
          }}
        />
      </div>

      {/* Formation flight area — above logo center */}
      <div style={{
        position: 'absolute',
        top: 'calc(50% - 145px)',
        left: '50%',
        transform: 'translateX(-50%)',
        width: 460,
        height: 90,
      }}>
        {/* Two staggered formations for continuous flow */}
        <FormationGroup delay={0} trailDelay={-0.25} />
        <FormationGroup delay={-1.75} trailDelay={-2.0} />
      </div>

      {/* "Loading ..." — below logo center */}
      <div style={{
        position: 'absolute',
        top: 'calc(50% + 70px)',
        left: '50%',
        transform: 'translateX(-50%)',
        color: 'rgba(200, 215, 230, 0.6)',
        fontSize: 13,
        fontWeight: 300,
        letterSpacing: 4,
        textTransform: 'uppercase',
      }}>
        {t('app.loading')}
      </div>
    </div>
  );
}
