import os
import platform
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import subprocess
import sys

PACKAGE_NAME = "pose_extractor"

options = {'--debug': 'OFF'}
if '--debug' in sys.argv:
    options['--debug'] = 'ON'


class CMakeExtension(Extension):
    def __init__(self, name, cmake_lists_dir=PACKAGE_NAME, **kwargs):
        Extension.__init__(self, name, sources=[], **kwargs)
        self.cmake_lists_dir = os.path.abspath(cmake_lists_dir)


class CMakeBuild(build_ext):
    def build_extensions(self):
        try:
            subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError('Cannot find CMake executable')

        ext = self.extensions[0]
        build_dir = os.path.abspath(os.path.join(PACKAGE_NAME, 'build'))
        if not os.path.exists(build_dir):
            os.mkdir(build_dir)
        tmp_dir = os.path.join(build_dir, 'tmp')
        if not os.path.exists(tmp_dir):
            os.mkdir(tmp_dir)

        cfg = 'Debug' if options['--debug'] == 'ON' else 'Release'

        cmake_args = [
            '-DCMAKE_BUILD_TYPE={}'.format(cfg),
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), build_dir),
            '-DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), tmp_dir),
            '-DPYTHON_EXECUTABLE={}'.format(sys.executable)
        ]

        subprocess.check_call(['cmake', ext.cmake_lists_dir] + cmake_args, cwd=tmp_dir)
        subprocess.check_call(['cmake', '--build', '.', '--config', cfg], cwd=tmp_dir)


setup(name=PACKAGE_NAME,
      packages=[PACKAGE_NAME],
      version='1.0',
      description='fast 2d pose extractor in openpose',
      ext_modules=[CMakeExtension(PACKAGE_NAME)],
      cmdclass={'build_ext': CMakeBuild})
