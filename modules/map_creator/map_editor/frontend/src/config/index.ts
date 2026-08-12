const runtimeConfig = typeof window !== 'undefined' ? window.MAP_EDITOR_CONFIG : undefined;
const backendPort = runtimeConfig?.backendPort || process.env.REACT_APP_MAP_BACKEND_PORT || '58000';
const configuredBackendHost = runtimeConfig?.backendHost || process.env.REACT_APP_MAP_BACKEND_HOST;
const devHost =
    typeof window !== 'undefined' && window.location
        ? `${window.location.hostname}:${backendPort}`
        : `localhost:${backendPort}`;

function normalizeHost(host: string) {
    return host
        .replace(/^https?:\/\//, '')
        .replace(/^wss?:\/\//, '')
        .replace(/\/$/, '');
}

export const baseHttpURL = normalizeHost(
    configuredBackendHost || (process.env.NODE_ENV === 'production' ? window.location.host : devHost),
);
export default {
    baseHttpURL,
    port: `${backendPort}`,
    timeout: 30000,
    connectionTimeout: 30000,
};
