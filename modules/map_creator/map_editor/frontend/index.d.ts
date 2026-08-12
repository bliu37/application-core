declare module '*.svg' {
    const content: string;
    export default content;
}
declare module '*.png' {
    const content: string;
    export default content;
}

interface Window {
    MAP_EDITOR_CONFIG?: {
        backendHost?: string;
        backendPort?: string | number;
    };
}
