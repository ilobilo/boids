# Copyright (C) 2024 ilobilo

project('boids', ['c', 'cpp'],
    default_options : [
        'cpp_std=c++17',
        'c_std=c17'
    ]
)

sources = files(
    'src/main.cpp',
    'src/boids.cpp',
    'src/trackball.cpp'
)

includes = include_directories('src')

executable('boids',
    dependencies : [
        dependency('fmt'),
        dependency('glfw3'),
        dependency('glm'),
        dependency('gl'),
        dependency('imgui', default_options : [
            'opengl=enabled',
            'glfw=enabled'
        ])
    ],
    sources : sources,
    include_directories : includes,
    cpp_args : [
        '-Wall',
        '-Wextra'
    ]
)