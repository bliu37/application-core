const fs = require('fs');
const path = require('path');

const projectRoot = path.resolve(__dirname, '..');
const defaultConfigPath = path.join(__dirname, 'server.config.json');
const configPath = process.env.MAP_EDITOR_CONFIG
  ? path.resolve(process.env.MAP_EDITOR_CONFIG)
  : defaultConfigPath;

const pathFields = [
  'baseMapRoot',
  'editorMapRoot',
  'releaseRoot',
  'converterBinary',
  'vehicleConfigPath',
  'frontendBuildDir',
];

const defaults = {
  port: 58000,
  baseMapRoot: path.join(projectRoot, 'data', 'base_map'),
  editorMapRoot: path.join(projectRoot, 'data', 'editor_map'),
  releaseRoot: path.join(projectRoot, 'data', 'released_map'),
  converterBinary: path.join(
    projectRoot,
    'bazel-bin',
    'modules',
    'private_tools',
    'map_tool',
    process.platform === 'win32' ? 'editor_map_converter.exe' : 'editor_map_converter'
  ),
  vehicleConfigPath: '/apollo/modules/common/data/vehicle_param.pb.txt',
  skipValidation: false,
  serveFrontend: true,
  frontendBuildDir: path.join(projectRoot, 'frontend', 'build', 'map_editor_frontend'),
};

function readJsonConfig(filePath) {
  if (!fs.existsSync(filePath)) {
    return {};
  }
  try {
    const raw = fs.readFileSync(filePath, 'utf8');
    return JSON.parse(raw);
  } catch (error) {
    console.warn(`[map-editor-backend] Failed to parse ${filePath}:`, error);
    return {};
  }
}

function toBoolean(value) {
  if (typeof value === 'boolean') {
    return value;
  }
  if (typeof value !== 'string') {
    return value;
  }
  return ['1', 'true', 'yes', 'on'].includes(value.toLowerCase());
}

function toNumber(value) {
  if (value === undefined || value === null || value === '') {
    return value;
  }
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : value;
}

function resolvePath(value, baseDir) {
  if (typeof value !== 'string' || value.length === 0) {
    return value;
  }
  if (path.isAbsolute(value)) {
    return value;
  }
  return path.resolve(baseDir, value);
}

function resolvePathFields(config, baseDir) {
  const resolved = { ...config };
  pathFields.forEach((field) => {
    if (resolved[field]) {
      resolved[field] = resolvePath(resolved[field], baseDir);
    }
  });
  return resolved;
}

function envOverrides() {
  const env = process.env;
  const overrides = {
    port: toNumber(env.MAP_EDITOR_PORT || env.PORT),
    baseMapRoot: env.MAP_EDITOR_BASE_MAP_ROOT,
    editorMapRoot: env.MAP_EDITOR_EDITOR_MAP_ROOT,
    releaseRoot: env.MAP_EDITOR_RELEASE_ROOT,
    converterBinary: env.MAP_EDITOR_CONVERTER_BINARY,
    vehicleConfigPath: env.MAP_EDITOR_VEHICLE_CONFIG_PATH,
    skipValidation:
      env.MAP_EDITOR_SKIP_VALIDATION === undefined
        ? undefined
        : toBoolean(env.MAP_EDITOR_SKIP_VALIDATION),
    serveFrontend:
      env.MAP_EDITOR_SERVE_FRONTEND === undefined
        ? undefined
        : toBoolean(env.MAP_EDITOR_SERVE_FRONTEND),
    frontendBuildDir: env.MAP_EDITOR_FRONTEND_BUILD_DIR,
  };

  Object.keys(overrides).forEach((key) => {
    if (overrides[key] === undefined || overrides[key] === '') {
      delete overrides[key];
    }
  });

  return resolvePathFields(overrides, projectRoot);
}

const userConfig = resolvePathFields(readJsonConfig(configPath), path.dirname(configPath));
const merged = {
  ...defaults,
  ...userConfig,
  ...envOverrides(),
};

module.exports = merged;
