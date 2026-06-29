## libfprint with Goodix 538d (goodixtls53xd) support
##
## Drop-in replacement for Fedora's libfprint that adds the goodixtls53xd
## driver (Goodix 27c6:538d) using a real TLS-PSK capture backend and the
## SIGFM (SIFT/OpenCV) matcher. Based on upstream libfprint 1.94.10 plus the
## work in https://github.com/lbssousa/libfprint (tag below).
##
## The Release is deliberately high so this supersedes Fedora's stock
## libfprint (1.94.10-5.fcNN) on the same Version. Rebase onto the next
## upstream Version when Fedora moves past 1.94.10.

# GitHub release tag and the directory name inside its archive (GitHub strips
# the leading "v" from vX.Y.Z-style tags).
%global goodix_tag v1.94.10-goodix538d
%global goodix_dir %{name}-1.94.10-goodix538d

Name:           libfprint
Version:        1.94.10
Release:        100.goodix538d%{?dist}
Summary:        Toolkit for fingerprint scanner (with Goodix 538d support)

# Most of the code is LGPL-2.1-or-later; libfprint/nbis is NIST-PD.
# The vendored SIGFM matcher (libfprint/sigfm) is LGPL-2.1-or-later.
License:        LGPL-2.1-or-later AND NIST-PD
URL:            https://github.com/lbssousa/libfprint
Source0:        %{url}/archive/refs/tags/%{goodix_tag}.tar.gz#/%{name}-%{goodix_tag}.tar.gz

BuildRequires:  meson
BuildRequires:  gcc
BuildRequires:  gcc-c++
BuildRequires:  openssl-devel
# Goodix 538d SIGFM matcher
BuildRequires:  pkgconfig(opencv4) >= 4.5.0
BuildRequires:  pkgconfig(glib-2.0) >= 2.50
BuildRequires:  pkgconfig(gio-2.0) >= 2.44.0
BuildRequires:  pkgconfig(gusb) >= 0.3.0
BuildRequires:  pkgconfig(nss)
BuildRequires:  pkgconfig(pixman-1)
BuildRequires:  gtk-doc
BuildRequires:  libgudev-devel
# For the udev.pc to install the rules
BuildRequires:  systemd
BuildRequires:  gobject-introspection-devel
# For internal CI tests; umockdev 0.13.2 has an important locking fix
BuildRequires:  python3-cairo python3-gobject cairo-devel
BuildRequires:  umockdev >= 0.13.2

# Runtime dependency of the SIGFM matcher in the goodixtls53xd driver
Requires:       opencv

%description
libfprint offers support for consumer fingerprint reader devices.

This build adds the goodixtls53xd driver for the Goodix 27c6:538d sensor
(real TLS-PSK capture + SIGFM/OpenCV matching) on top of upstream 1.94.10.

%package        devel
Summary:        Development files for %{name}
Requires:       %{name}%{?_isa} = %{version}-%{release}

%description    devel
The %{name}-devel package contains libraries and header files for
developing applications that use %{name}.

%package        tests
Summary:        Tests for the %{name} package
Requires:       %{name}%{?_isa} = %{version}-%{release}

%description tests
The %{name}-tests package contains tests that can be used to verify
the functionality of the installed %{name} package.

%prep
%autosetup -n %{goodix_dir}

%build
# "all" includes the default drivers (with goodixtls53xd) plus the virtual
# drivers used by the integration tests.
%meson -Ddrivers=all
%meson_build

%install
%meson_install

%ldconfig_scriptlets

# %%check is intentionally skipped: upstream/Fedora currently disable the
# installed tests due to a pygobject 3.52 incompatibility.

%files
%license COPYING
%doc NEWS THANKS AUTHORS README.md
%{_libdir}/*.so.*
%{_libdir}/girepository-1.0/*.typelib
%{_udevhwdbdir}/60-autosuspend-libfprint-2.hwdb
%{_udevrulesdir}/70-libfprint-2.rules
%{_datadir}/metainfo/org.freedesktop.libfprint.metainfo.xml

%files devel
%doc HACKING.md
%{_includedir}/*
%{_libdir}/*.so
%{_libdir}/pkgconfig/%{name}-2.pc
%{_datadir}/gir-1.0/*.gir
%{_datadir}/gtk-doc/html/libfprint-2/

%files tests
%{_libexecdir}/installed-tests/libfprint-2/
%{_datadir}/installed-tests/libfprint-2/

%changelog
* Sun Jun 28 2026 Laercio de Sousa <laercio@sivali.sousa.nom.br> - 1.94.10-100.goodix538d
- Fork of Fedora's libfprint 1.94.10 adding the goodixtls53xd driver
  (Goodix 27c6:538d): real TLS-PSK capture backend + SIGFM/OpenCV matching.
- Add opencv4 BuildRequires and opencv runtime Requires.
- Build with -Ddrivers=all (includes goodixtls53xd).
