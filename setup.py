  # Always prefer setuptools over distutils
  from setuptools import setup, find_packages

  setup(
      name='precland',

      # Versions should comply with PEP440.  For a discussion on single-sourcing
      # the version across setup.py and the project code, see
      # https://packaging.python.org/en/latest/single_source_version.html
      version='0.0.1',

      description='Precision land for arducopter',

      # The project's main homepage.
      url='https://github.com/3drobotics/precland',

      # Choose your license
      license='GPLv3',

      # See https://pypi.python.org/pypi?%3Aaction=list_classifiers
      classifiers=[
          #   3 - Alpha
          #   4 - Beta
          #   5 - Production/Stable
          'Development Status :: 3 - Alpha',

          # Indicate who your project is intended for
          'Intended Audience :: Developers',
          'Topic :: Software Development :: Build Tools',

          # Pick your license as you wish (should match "license" above)
          'License :: OSI Approved :: GPLv3 License',

          'Programming Language :: Python :: 2.7',
      ],

      # What does your project relate to?
      keywords='precland',

      # You can just specify the packages manually here if your project is
      # simple. Or you can use find_packages().
      packages=find_packages(),

      # List run-time dependencies here.  These will be installed by pip when
      # your project is installed. For an analysis of "install_requires" vs pip's
      # requirements files see:
      # https://packaging.python.org/en/latest/requirements.html
      install_requires=[
          'numpy',
          'pymavlink',
          'pyserial',
          'python-opencv',
          'cv_utils'
      ],
      dependency_links=[
          'git+https://git@github.com/djnugent/cv_utils.git',
      ],
      entry_points={
          'console_scripts': [
              'precland = precland.sh'
          ]
      },

  )
