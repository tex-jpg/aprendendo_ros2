from setuptools import find_packages, setup

package_name = 'meu_primeiro_pacote'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/meu_primeiro_launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henrique Teixeira',
    maintainer_email='uniehteixeira@fei.edu.br',
    description='Aprendendo a mexer no ros',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'meu_primeiro_no = meu_primeiro_pacote.meu_primeiro_no:main',
            'no_com_classe = meu_primeiro_pacote.no_com_classe:main',
            'talker = meu_primeiro_pacote.talker:main',
            'listener = meu_primeiro_pacote.listener:main',
            'r2d2 = meu_primeiro_pacote.r2d2:main',
            'r2d2_controle = meu_primeiro_pacote.r2d2_controle:main'
            'wavefront = meu_primeiro_pacote.wavefront:main'

        ],
    },
)
