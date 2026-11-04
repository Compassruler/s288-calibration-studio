# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_data_files
from PyInstaller.utils.hooks import collect_dynamic_libs

datas = [('C:/Users/Administrator/Desktop/microduck/个人开发/s288-calibration-studio/assets', 'assets'), ('C:/Users/Administrator/Desktop/microduck/个人开发/s288-calibration-studio/configs/examples', 'configs/examples')]
binaries = [('C:/ProgramData/miniconda3/envs/microduck/Library/bin/tcl90.dll', '.'), ('C:/ProgramData/miniconda3/envs/microduck/Library/bin/tcl9tk90.dll', '.'), ('C:/ProgramData/miniconda3/envs/microduck/Library/bin/libtommath.dll', '.'), ('C:/ProgramData/miniconda3/envs/microduck/Library/bin/zlib1.dll', '.'), ('C:/ProgramData/miniconda3/envs/microduck/Library/bin/ffi.dll', '.'), ('C:/ProgramData/miniconda3/envs/microduck/Library/bin/libmpdec-4.dll', '.')]
datas += collect_data_files('mujoco')
datas += collect_data_files('glfw')
binaries += collect_dynamic_libs('mujoco')
binaries += collect_dynamic_libs('glfw')


a = Analysis(
    ['C:/Users/Administrator/Desktop/microduck/个人开发/s288-calibration-studio/app.py'],
    pathex=['C:/Users/Administrator/Desktop/microduck/个人开发/s288-calibration-studio'],
    binaries=binaries,
    datas=datas,
    hiddenimports=['robot_view', 'serial.tools.list_ports', 'mujoco.rendering.classic.renderer', 'mujoco.rendering.classic.gl_context', 'mujoco.glfw'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['pytest', 'IPython', 'matplotlib', 'pandas', 'scipy', 'jax', 'torch', 'tensorflow', 'OpenGL'],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='S288Studio',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    contents_directory='.',
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='S288Studio',
)
