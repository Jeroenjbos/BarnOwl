try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

setup(name='BarnOwl',
      description='Motion platform',
      author='Ronny Eichler',
      author_email='ronny.eichler@gmail.com',
      version='0.0.1',
      install_requires=['cobs', 'pyerial'],
      packages=['barnowl'],
  )
