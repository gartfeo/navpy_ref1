import { execSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { viteStaticCopy } from 'vite-plugin-static-copy'
import { manifestVersionPlugin } from './manifestVersion.js'

const PUBLIC_DIR = fileURLToPath(new URL('./public', import.meta.url))

// Per-chat ports come from the launcher (scripts/gcs_launch.py) via env so
// multiple GCS stacks can run without colliding. Fall back to the classic
// 3000/8000 defaults for a plain `npm run dev`.
const FRONTEND_PORT = Number(process.env.PORT) || 3000
const BACKEND_PORT = Number(process.env.VITE_BACKEND_PORT) || 8000
const BACKEND_HTTP = `http://localhost:${BACKEND_PORT}`
const BACKEND_WS = `ws://localhost:${BACKEND_PORT}`

// Git branch shown in the app title so it's obvious which version is running.
function gitBranch() {
  if (process.env.VITE_GIT_BRANCH) return process.env.VITE_GIT_BRANCH
  try {
    return execSync('git rev-parse --abbrev-ref HEAD').toString().trim()
  } catch {
    return ''
  }
}

export default defineConfig({
  plugins: [
    react(),
    viteStaticCopy({
      targets: [
        {
          src: 'node_modules/cesium/Build/Cesium/*',
          dest: 'Cesium',
        },
      ],
    }),
    manifestVersionPlugin(PUBLIC_DIR),
  ],
  define: {
    CESIUM_BASE_URL: JSON.stringify('/Cesium/'),
    __APP_BRANCH__: JSON.stringify(gitBranch()),
  },
  optimizeDeps: {
    exclude: ['cesium'],
    include: [
      'mersenne-twister', 'urijs', 'grapheme-splitter', 'bitmap-sdf',
      'lerc', 'kdbush', 'rbush', 'earcut', 'pako', 'draco3d',
      'potpack', 'nosleep.js',
    ],
  },
  build: {
    commonjsOptions: {
      transformMixedEsModules: true,
      defaultIsModuleExports: true,
    },
  },
  // Vitest picks up this `test` block from the shared Vite config. Component
  // tests run under jsdom; the setup file installs RTL cleanup, timer/mock
  // restore, and the react-i18next stub. Scope discovery to *.test.{js,jsx}
  // so it never sweeps the pytest-driven tests/ suite.
  test: {
    environment: 'jsdom',
    setupFiles: ['./src/test/setup.js'],
    include: ['src/**/*.test.{js,jsx}'],
    css: false,
  },
  server: {
    host: true,
    port: FRONTEND_PORT,
    proxy: {
      '/api': BACKEND_HTTP,
      '/ws': {
        target: BACKEND_WS,
        ws: true,
        configure: (proxy) => {
          proxy.on('proxyReqWs', (_proxyReq, _req, socket) => {
            socket.on('error', () => {});
          });
          proxy.on('open', (proxySocket) => {
            proxySocket.on('error', () => {});
          });
          // Vite adds its own error handlers after configure returns;
          // defer removal so only our silent handler remains.
          setTimeout(() => {
            proxy.removeAllListeners('error');
            proxy.on('error', () => {});
          }, 0);
        },
      },
      '/health': BACKEND_HTTP,
    },
  },
  // `vite preview` reads this separately from `server` above — needed so a
  // production-build preview also proxies to the backend (used for a quick
  // manual timing check of dev vs. prod-bundle Cesium load time).
  preview: {
    host: true,
    port: FRONTEND_PORT,
    proxy: {
      '/api': BACKEND_HTTP,
      '/ws': { target: BACKEND_WS, ws: true },
      '/health': BACKEND_HTTP,
    },
  },
})
