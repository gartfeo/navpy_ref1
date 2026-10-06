import React from 'react';
import ReactDOM from 'react-dom/client';
import './i18n';
import App from './App';

// Neutral base tab title; App appends the running branch in dev mode only
// (see App.jsx), so operators in non-dev mode never see the branch name.
document.title = 'AAS GCS';

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);

// Register service worker for offline tile caching
if ('serviceWorker' in navigator) {
  window.addEventListener('load', () => {
    navigator.serviceWorker.register('/sw.js').catch(() => {});
  });
}
