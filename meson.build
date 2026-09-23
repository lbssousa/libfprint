project('libfprint', [ 'c', 'cpp' ],
    version: '1.94.10',
    license: 'LGPLv2.1+',
    default_options: [
        'buildtype=debugoptimized',
        'warning_level=1',
        'c_std=gnu99',
    ],
    meson_version: '>= 0.59.0')

fs = import('fs')
gnome = import('gnome')

libfprint_conf = configuration_data()
libfprint_conf.set_quoted('LIBFPRINT_VERSION', meson.project_version())

prefix = get_option('prefix')
libdir = prefix / get_option('libdir')
libexecdir = prefix / get_option('libexecdir')
datadir = prefix / get_option('datadir')

cc = meson.get_compiler('c')
cpp = meson.get_compiler('cpp')
host_system = host_machine.system()

libfprint_sanitizers = get_option('b_sanitize').split(',')
if libfprint_sanitizers == ['none']
  libfprint_sanitizers = []
endif

glib_min_version = '2.68'

glib_version_def = 'GLIB_VERSION_@0@_@1@'.format(
    glib_min_version.split('.')[0], glib_min_version.split('.')[1])
common_cflags = cc.get_supported_arguments([
    '-Wcast-align',
    '-Wformat-nonliteral',
    '-Wformat-security',
    '-Wformat=2',
    '-Wignored-qualifiers',
    '-Wlogical-op',
    '-Wmissing-declarations',
    '-Wmissing-format-attribute',
    '-Wmissing-include-dirs',
    '-Wmissing-noreturn',
    '-Wpointer-arith',
    '-Wshadow',
    '-Wswitch-enum',
    '-Wtype-limits',
    '-Wundef',
    '-Wunused',
    '-Werror=address',
    '-Werror=array-bounds',
    '-Werror=empty-body',
    '-Werror=init-self',
    '-Werror=int-to-pointer-cast',
    '-Werror=main',
    '-Werror=missing-braces',
    '-Werror=nonnull',
    '-Werror=redundant-decls',
    '-Werror=return-type',
    '-Werror=sequence-point',
    '-Werror=trigraphs',
    '-Werror=write-strings',
    '-fno-strict-aliasing',
    '-DGLIB_VERSION_MIN_REQUIRED=' + glib_version_def,
    '-DGLIB_VERSION_MAX_ALLOWED=' + glib_version_def,
    '-D_GNU_SOURCE',
    '-DG_LOG_DOMAIN="@0@"'.format(meson.project_name()),
])
c_cflags = cc.get_supported_arguments([
    '-Wimplicit-function-declaration',
    '-Wmissing-prototypes',
    '-Wnested-externs',
    '-Wold-style-definition',
    '-Wstrict-prototypes',
    '-Werror=implicit',
    '-Werror=pointer-to-int-cast',
])
add_project_arguments(common_cflags + c_cflags, language: 'c')
add_project_arguments(common_cflags, language: 'cpp')

# maintaining compatibility with the previous libtool versioning
# current = binary - interface
# revision = interface
soversion = 2
current = 0
revision = 0
libversion = '@0@.@1@.@2@'.format(soversion, current, revision)
versioned_libname = meson.project_name() + '-' + soversion.to_string()

# Dependencies
glib_dep = dependency('glib-2.0', version: '>=' + glib_min_version)
gio_dep = dependency('gio-unix-2.0', version: '>=' + glib_min_version)
gobject_dep = dependency('gobject-2.0', version: '>=' + glib_min_version)
gmodule_dep = dependency('gmodule-2.0', version: '>=' + glib_min_version)
gusb_dep = dependency('gusb', version: '>= 0.2.0')
mathlib_dep = cc.find_library('m', required: false)

# The following dependencies are only used for tests
sh = find_program('sh', required: true)
cairo_dep = dependency('cairo', required: false)

# introspection scanning and Gio-2.0.gir
gobject_introspection = dependency('gobject-introspection-1.0', required: get_option('introspection'))

# SPI
have_spi = host_machine.system() == 'linux'

# Drivers
drivers = get_option('drivers').split(',')
virtual_drivers = [
    'virtual_image',
    'virtual_device',
    'virtual_device_storage',
]

default_drivers = [
    'upektc_img',
    'vfs5011',
    'vfs7552',
    'aes3500',
    'aes4000',
    'aes1610',
    'aes1660',
    'aes2660',
    'aes2501',
    'aes2550',
    'vfs101',
    'vfs301',
    'vfs0050',
    'etes603',
    'egis0570',
    'egismoc',
    'egis_etu905',
    'vcom5s',
    'synaptics',
    'elan',
    'elanmoc',
    'uru4000',
    'upektc',
    'upeksonly',
    'upekts',
    'goodixmoc',
    'goodixtls53xd',
    'nb1010',
    'fpcmoc',
    'realtek',
    'focaltech_moc',
    'mafpmoc',
    'secugen',
]

spi_drivers = [
    'elanspi'
]

if have_spi
    default_drivers += spi_drivers
endif

# FIXME: All the drivers should be fixed by adjusting the byte order.
# See https://gitlab.freedesktop.org/libfprint/libfprint/-/issues/236
endian_independent_drivers = virtual_drivers + [
    'aes1610',
    'aes1660',
    'aes2550',
    'aes2660',
    'aes3500',
    'aes4000',
    'egis0570',
    'egismoc',
    'egis_etu905',
    'elanmoc',
    'etes603',
    'focaltech_moc',
    'nb1010',
    'realtek',
    'synaptics',
    'upeksonly',
    'upektc',
    'upektc_img',
    'upekts',
    'vcom5s',
    'vfs101',
    'vfs7552',
]

all_drivers = default_drivers + virtual_drivers

if drivers == [ 'all' ]
    drivers = all_drivers
endif

if drivers == [ 'default' ]
    drivers = default_drivers
endif

enabled_spi_drivers = []
foreach driver : spi_drivers
    if driver in drivers
        enabled_spi_drivers += driver
    endif
endforeach

if enabled_spi_drivers.length() > 0 and not have_spi
    error('SPI drivers @0@ are not supported'.format(enabled_spi_drivers))
endif

driver_helper_mapping = {
    'aes1610' : [ 'aeslib' ],
    'aes1660' : [ 'aeslib', 'aesx660' ],
    'aes2501' : [ 'aeslib' ],
    'aes2550' : [ 'aeslib' ],
    'aes2660' : [ 'aeslib', 'aesx660' ],
    'aes3500' : [ 'aeslib', 'aes3k' ],
    'aes4000' : [ 'aeslib', 'aes3k' ],
    'uru4000' : [ 'openssl' ],
    'goodixtls53xd' : [ 'goodixtls', 'openssl' ],
    'elanspi' : [ 'udev' ],
    'virtual_image'          : [ 'virtual' ],
    'virtual_device'         : [ 'virtual' ],
    'virtual_device_storage' : [ 'virtual' ],
}

driver_helpers = []
foreach driver : drivers
    if driver in driver_helper_mapping
        foreach helper : driver_helper_mapping[driver]
            if helper not in driver_helpers
                driver_helpers += helper
            endif
        endforeach
    endif
endforeach


if drivers.length() == 0 or drivers[0] == ''
    error('Cannot build libfprint without drivers, please specify a valid value for the drivers option')
endif

if drivers == all_drivers or drivers == default_drivers
    default_drivers_are_enabled = true
else
    default_drivers_are_enabled = true
    foreach driver: default_drivers
        if driver not in drivers
            default_drivers_are_enabled = false
            break
        endif
    endforeach
endif

udev_rules = get_option('udev_rules')
install_udev_rules = udev_rules.enabled()

optional_deps = []

# Resolve extra dependencies
foreach i : driver_helpers
    foreach d, helpers : driver_helper_mapping
        if i in helpers
            driver = d
            break
        endif
    endforeach

    if i == 'aes3k'
        imaging_dep = dependency('pixman-1', required: false)
        if not imaging_dep.found()
            error('pixman is required for @0@ and possibly others'.format(driver))
        endif

        libfprint_conf.set10('HAVE_PIXMAN', true)
        optional_deps += imaging_dep
    elif i == 'openssl'
        openssl_dep = dependency('openssl', version: '>= 3.0', required: false)
        if not openssl_dep.found()
            error('OpenSSL is required for @0@ and possibly others'.format(driver))
        endif

        optional_deps += openssl_dep
    elif i == 'goodixtls'
        threads_dep = dependency('threads', required: false)
        if not threads_dep.found()
            error('threads is required for @0@ and possibly others'.format(driver))
        endif

        optional_deps += threads_dep

        # SIGFM matcher (OpenCV-based) used by the goodixtls53xd driver for
        # the tiny press captures where the default minutiae matcher fails.
        # OpenCV's pkg-config file is named after its major version
        # (opencv4.pc, opencv5.pc, ...); the API SIGFM uses hasn't changed
        # across that boundary, so accept whichever is installed.
        opencv_dep = dependency('opencv4', version: '>= 4.5.0', required: false)
        if not opencv_dep.found()
            opencv_dep = dependency('opencv5', version: '>= 5.0.0', required: false)
        endif
        if not opencv_dep.found()
            # No system OpenCV: build the small subset SIGFM actually needs
            # (core, imgproc, flann, geometry, features) from vendored
            # source, statically, so the driver doesn't require
            # opencv-devel/opencv5-devel to be installed.
            #
            # This shells out to OpenCV's OWN native CMake build via
            # run_command(), rather than going through Meson's
            # cmake.subproject() translator (tried first; kept failing
            # with "opencl_kernels_core.hpp: No such file or directory"
            # and equivalents for imgproc/features/geometry). Root cause:
            # those four modules unconditionally #include a generated
            # opencl_kernels_<module>.hpp — regardless of WITH_OPENCL —
            # produced by an add_custom_command() running
            # cmake/cl2cpp.cmake over each module's src/opencl/*.cl
            # sources. Meson's CMake-subproject support translates
            # target/library definitions but does not replicate arbitrary
            # custom commands, so the header silently never gets
            # generated and every .cpp that includes it fails to compile.
            # A real `cmake --build` has no such gap, since it runs
            # OpenCV's own custom commands itself. Verified end-to-end
            # (2026-09-22): the translator approach fails to build at
            # all; this one produces a working static libfprint-2.so
            # with zero runtime OpenCV/BLAS dependency.
            opencv_version = '5.0.0'
            # Matches subprojects/opencv.wrap (kept for documentation /
            # any future switch back to a wrap-aware build method).
            opencv_source_url = 'https://github.com/opencv/opencv/archive/refs/tags/' + opencv_version + '.tar.gz'
            opencv_source_hash = 'b0528f5a1d379d59d4701cb28c36e22214cc51cf64594e5b56f2d3e6c0233095'

            cmake_prog = find_program('cmake', required: true)
            curl_prog = find_program('curl', required: true)
            tar_prog = find_program('tar', required: true)
            sha256sum_prog = find_program('sha256sum', required: true)

            subprojects_dir = meson.project_source_root() / 'subprojects'
            opencv_src_dir = subprojects_dir / ('opencv-' + opencv_version)
            opencv_tarball = subprojects_dir / 'packagecache' / ('opencv-' + opencv_version + '.tar.gz')

            if not fs.is_dir(opencv_src_dir)
                fetch_sh = '''set -eu
mkdir -p "$(dirname "$2")" "$3"
if [ ! -e "$2" ]; then
    "$4" -L --fail -o "$2.tmp" "$1"
    mv "$2.tmp" "$2"
fi
echo "$5  $2" | "$6" -c -
"$7" -xzf "$2" -C "$3" --strip-components=1
'''
                fetch_result = run_command('sh', '-c', fetch_sh, 'fetch-opencv',
                    opencv_source_url, opencv_tarball, opencv_src_dir,
                    curl_prog.full_path(), opencv_source_hash,
                    sha256sum_prog.full_path(), tar_prog.full_path(),
                    check: true,
                )
            endif

            opencv_build_dir = meson.project_build_root() / 'opencv-native-build'
            opencv_install_dir = meson.project_build_root() / 'opencv-native-install'

            # CPU_BASELINE/CPU_DISPATCH are cleared because OpenCV's
            # per-file multi-variant CPU dispatch (the same
            # .dispatch.cpp compiled several times with different SIMD
            # flags) is more machinery than the tiny fingerprint
            # captures SIGFM matches need; a single baseline build is
            # plenty and keeps this fast.
            opencv_configure = run_command(cmake_prog,
                '-S', opencv_src_dir,
                '-B', opencv_build_dir,
                '-G', 'Ninja',
                '-DBUILD_LIST=core,flann,geometry,imgproc,features',
                '-DBUILD_SHARED_LIBS=OFF',
                '-DBUILD_TESTS=OFF',
                '-DBUILD_PERF_TESTS=OFF',
                '-DBUILD_EXAMPLES=OFF',
                '-DBUILD_DOCS=OFF',
                '-DBUILD_opencv_apps=OFF',
                '-DBUILD_JAVA=OFF',
                '-DBUILD_opencv_python2=OFF',
                '-DBUILD_opencv_python3=OFF',
                '-DBUILD_opencv_js=OFF',
                '-DBUILD_opencv_objc=OFF',
                '-DWITH_1394=OFF',
                '-DWITH_FFMPEG=OFF',
                '-DWITH_GSTREAMER=OFF',
                '-DWITH_GTK=OFF',
                '-DWITH_QT=OFF',
                '-DWITH_V4L=OFF',
                '-DWITH_OPENCL=OFF',
                '-DWITH_CUDA=OFF',
                '-DWITH_VTK=OFF',
                '-DWITH_OPENEXR=OFF',
                '-DWITH_JPEG=OFF',
                '-DWITH_PNG=OFF',
                '-DWITH_TIFF=OFF',
                '-DWITH_WEBP=OFF',
                '-DWITH_OPENJPEG=OFF',
                '-DWITH_JASPER=OFF',
                '-DWITH_IPP=OFF',
                '-DWITH_ITT=OFF',
                '-DWITH_LAPACK=OFF',
                '-DWITH_EIGEN=OFF',
                '-DWITH_PROTOBUF=OFF',
                '-DWITH_QUIRC=OFF',
                '-DCPU_BASELINE=',
                '-DCPU_DISPATCH=',
                '-DCV_DISABLE_OPTIMIZATION=ON',
                '-DCMAKE_POSITION_INDEPENDENT_CODE=ON',
                '-DCMAKE_BUILD_TYPE=Release',
                '-DCMAKE_INSTALL_PREFIX=' + opencv_install_dir,
                check: true,
            )
            opencv_build = run_command(cmake_prog, '--build', opencv_build_dir, '--parallel', check: true)
            opencv_install = run_command(cmake_prog, '--install', opencv_build_dir, check: true)

            opencv_libdir = opencv_install_dir / 'lib'
            if not fs.is_dir(opencv_libdir)
                opencv_libdir = opencv_install_dir / 'lib64'
            endif

            # Link order matters for static libs: dependents before their
            # dependencies (features -> {imgproc, flann} -> core).
            opencv_link_args = []
            foreach m : ['features', 'imgproc', 'flann', 'geometry', 'core']
                opencv_link_args += opencv_libdir / ('libopencv_' + m + '.a')
            endforeach

            # core's persistence.cpp (XML/YAML/JSON (de)serialization)
            # calls zlib's gz* stream functions unconditionally — there's
            # no WITH_ZLIB switch in the option list above to turn that
            # off. Whether CMake vendored its own 3rdparty/zlib or found
            # a system one depends on what else happens to be installed
            # in the build environment (confirmed: it varies), and a
            # static .a never carries its own transitive link
            # dependencies forward — so resolve zlib explicitly here
            # rather than relying on either path.
            zlib_dep = dependency('zlib', required: true)

            # CMake's GNUInstallDirs names the include subdir after the
            # major version (opencv4/ for 4.x, opencv5/ for 5.x, ...).
            #
            # include_directories() can't be used here: it unconditionally
            # rejects any absolute path argument ("Tried to form an
            # absolute path to a dir in the source tree"), and
            # opencv_install_dir is necessarily absolute (it's under
            # meson.project_build_root(), only known at configure time).
            # -isystem via compile_args is the standard workaround, and
            # keeps the same "system header" treatment (silences warnings
            # from OpenCV's own headers) that is_system: true gave us.
            opencv_major = opencv_version.split('.')[0]
            opencv_dep = declare_dependency(
                link_args: opencv_link_args,
                dependencies: [zlib_dep],
                compile_args: ['-isystem' + (opencv_install_dir / 'include' / ('opencv' + opencv_major))],
            )
        endif

        optional_deps += opencv_dep
    elif i == 'udev'
        install_udev_rules = true

        gudev_dep = dependency('gudev-1.0', required: false)
        if not gudev_dep.found()
            error('udev is required for SPI support')
        endif

        libfprint_conf.set10('HAVE_UDEV', true)

        optional_deps += gudev_dep
    endif
endforeach

if udev_rules.disabled()
    install_udev_rules = false
endif

if install_udev_rules
    udev_rules_dir = get_option('udev_rules_dir')
    if udev_rules_dir == 'auto'
        udev_dep = dependency('udev')
        udev_rules_dir = udev_dep.get_variable(pkgconfig: 'udevdir') + '/rules.d'
    endif
endif

supported_drivers = []
foreach driver: drivers
    if build_machine.endian() == 'little' or driver in endian_independent_drivers
        supported_drivers += driver
    else
        warning('Driver @0@ is not supported by big endian cpu @1@. Please, fix it!'.format(
            driver, build_machine.cpu()))
    endif
endforeach

if default_drivers_are_enabled and supported_drivers != drivers
    default_drivers_are_enabled = false
endif

root_inc = include_directories('.')

udev_hwdb = get_option('udev_hwdb')
if not udev_hwdb.disabled()
    # systemd v248 includes our autosuspend rules
    udev_versioned_dep = dependency('udev', version: '>= 248', required: false)
    udev_hwdb_dir = get_option('udev_hwdb_dir')
    if udev_hwdb.auto()
        if udev_versioned_dep.found()
            udev_hwdb_dir = ''
        endif
    else
        if udev_versioned_dep.found()
            warning('udev hwdb will be installed by both systemd and libfprint')
        endif
    endif

    if udev_hwdb_dir == 'auto'
        udev_dep = dependency('udev')
        udev_hwdb_dir = udev_dep.get_variable(pkgconfig: 'udevdir') + '/hwdb.d'
    endif
else
    udev_hwdb_dir = ''
endif

if get_option('gtk-examples')
    gtk_dep = dependency('gtk+-3.0', required: false)
    if not gtk_dep.found()
        error('GTK+ 3.x is required for GTK+ examples')
    endif
endif

# Some dependency resolving happens inside here
subdir('libfprint')

configure_file(output: 'config.h', configuration: libfprint_conf)

if get_option('doc')
    subdir('doc')
endif
if get_option('gtk-examples')
    subdir('demo')
endif

subdir('data')
subdir('tests')

subdir('examples')

pkgconfig = import('pkgconfig')
pkgconfig.generate(
    name: versioned_libname,
    description: 'Generic C API for fingerprint reader access',
    version: meson.project_version(),
    libraries: libfprint,
    requires: [gio_dep, gobject_dep],
    subdirs: versioned_libname,
    filebase: versioned_libname,
)

summary({'Drivers': drivers, }, section: 'Drivers')
