from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'as_bugs'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:
        
        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                    
                else:
                    paths_dict[install_path] = [file_path]
                
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['models/', 'launch/', 'worlds/', 'config/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='branimir',
    maintainer_email='branimir@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parse_urdf2marker = as_bugs.parse_urdf2marker:main',
            'tracker = as_bugs.tracker:main',
            'scan_to_range = as_bugs.scan_to_range:main',
            'bugx = as_bugs.bugx:main',
        ],
    },
)
