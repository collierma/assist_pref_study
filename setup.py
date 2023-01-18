from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_info = generate_distutils_setup()
package_info['packages'] = ['assist_pref_grasp_demo']
package_info['package_dir'] = {'':'src'}
package_info['install_requires'] = []

setup(**package_info)