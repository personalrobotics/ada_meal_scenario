#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_info = generate_distutils_setup()
package_info['packages'] = ['ada_meal_scenario', 
                            'ada_meal_scenario.actions']
package_info['package_dir'] = {'ada_meal_scenario':'src'}
package_info['install_requires'] = []

setup(**package_info)
