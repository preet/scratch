TEMPLATE = lib
CONFIG -= qt
CONFIG += plugin # hack to avoid sonames
TARGET = marisa

# ======================================================= #
# paths
# ======================================================= #
PATH_MARISA=/home/preet/Packages/marisa-0.2.4/lib

# ======================================================= #
# options
# ======================================================= #
#CONFIG += enable_popcnt
#CONFIG += enable_sse2
#CONFIG += enable_sse3
#CONFIG += enable_ssse3
#CONFIG += enable_sse4_1
#CONFIG += enable_sse4_2
#CONFIG += enable_sse4
#CONFIG += enable_sse4a

# ======================================================= #

enable_popcnt {
    CONFIG+=enable_sse3
}
enable_sse4a {
    CONFIG+=enable_popcnt
    CONFIG+=enable_sse3
}
enable_sse4 {
    CONFIG+=enable_popcnt
    CONFIG+=enable_sse4_2
}
enable_sse4_2 {
    CONFIG+=enable_popcnt
    CONFIG+=enable_sse4_1
}
enable_sse4_1 {
    CONFIG+=enable_ssse3
}
enable_ssse3 {
    CONFIG+=enable_sse3
}
enable_sse3 {
    CONFIG+=enable_sse2
}

# ======================================================= #

enable_popcnt {
    QMAKE_CXXFLAGS += -DMARISA_USE_POPCNT -mpopcnt
}
enable_sse4a {
    QMAKE_CXXFLAGS += -DMARISA_USE_SSE4A -msse4a
}
enable_sse4 {
    QMAKE_CXXFLAGS += -DMARISA_USE_SSE4 -msse4
}
enable_sse4_2 {
    QMAKE_CXXFLAGS +=-DMARISA_USE_SSE4_2 -msse4.2
}
enable_sse4_1 {
    QMAKE_CXXFLAGS += -DMARISA_USE_SSE4_1 -msse4.1
}
enable_ssse3 {
    QMAKE_CXXFLAGS += -DMARISA_USE_SSSE3 -mssse3
}
enable_sse3 {
    QMAKE_CXXFLAGS += -DMARISA_USE_SSE3 -msse3
}
enable_sse2 {
    QMAKE_CXXFLAGS += -DMARISA_USE_SSE2 -msse2
}

# ======================================================= #
# source
# ======================================================= #
INCLUDEPATH += $${PATH_MARISA}

# install headers
HEADERS += \
  $${PATH_MARISA}/marisa/base.h \
  $${PATH_MARISA}/marisa/exception.h \
  $${PATH_MARISA}/marisa/scoped-ptr.h \
  $${PATH_MARISA}/marisa/scoped-array.h \
  $${PATH_MARISA}/marisa/key.h \
  $${PATH_MARISA}/marisa/keyset.h \
  $${PATH_MARISA}/marisa/query.h \
  $${PATH_MARISA}/marisa/agent.h \
  $${PATH_MARISA}/marisa/stdio.h \
  $${PATH_MARISA}/marisa/iostream.h \
  $${PATH_MARISA}/marisa/trie.h

# dont install these headers
HEADERS += \
  $${PATH_MARISA}/marisa/grimoire/intrin.h \
  $${PATH_MARISA}/marisa/grimoire/io.h \
  $${PATH_MARISA}/marisa/grimoire/io/mapper.h \
  $${PATH_MARISA}/marisa/grimoire/io/reader.h \
  $${PATH_MARISA}/marisa/grimoire/io/writer.h \
  $${PATH_MARISA}/marisa/grimoire/vector.h \
  $${PATH_MARISA}/marisa/grimoire/vector/pop-count.h \
  $${PATH_MARISA}/marisa/grimoire/vector/rank-index.h \
  $${PATH_MARISA}/marisa/grimoire/vector/vector.h \
  $${PATH_MARISA}/marisa/grimoire/vector/flat-vector.h \
  $${PATH_MARISA}/marisa/grimoire/vector/bit-vector.h \
  $${PATH_MARISA}/marisa/grimoire/algorithm.h \
  $${PATH_MARISA}/marisa/grimoire/algorithm/sort.h \
  $${PATH_MARISA}/marisa/grimoire/trie.h \
  $${PATH_MARISA}/marisa/grimoire/trie/config.h \
  $${PATH_MARISA}/marisa/grimoire/trie/header.h \
  $${PATH_MARISA}/marisa/grimoire/trie/key.h \
  $${PATH_MARISA}/marisa/grimoire/trie/range.h \
  $${PATH_MARISA}/marisa/grimoire/trie/entry.h \
  $${PATH_MARISA}/marisa/grimoire/trie/tail.h \
  $${PATH_MARISA}/marisa/grimoire/trie/cache.h \
  $${PATH_MARISA}/marisa/grimoire/trie/history.h \
  $${PATH_MARISA}/marisa/grimoire/trie/state.h \
  $${PATH_MARISA}/marisa/grimoire/trie/louds-trie.h

SOURCES += \
  $${PATH_MARISA}/marisa/keyset.cc \
  $${PATH_MARISA}/marisa/agent.cc \
  $${PATH_MARISA}/marisa/trie.cc \
  $${PATH_MARISA}/marisa/grimoire/io/mapper.cc \
  $${PATH_MARISA}/marisa/grimoire/io/reader.cc \
  $${PATH_MARISA}/marisa/grimoire/io/writer.cc \
  $${PATH_MARISA}/marisa/grimoire/vector/bit-vector.cc \
  $${PATH_MARISA}/marisa/grimoire/trie/tail.cc \
  $${PATH_MARISA}/marisa/grimoire/trie/louds-trie.cc

# ======================================================= #
# install
# ======================================================= #
PATH_INSTALL = /home/preet/Dev/env/sys/marisa
target.path = $${PATH_INSTALL}/lib

header_single.path = $${PATH_INSTALL}/include
header_single.files = $${PATH_MARISA}/marisa.h

header_files.path = $${PATH_INSTALL}/include/marisa
header_files.files = \
  $${PATH_MARISA}/marisa/base.h \
  $${PATH_MARISA}/marisa/exception.h \
  $${PATH_MARISA}/marisa/scoped-ptr.h \
  $${PATH_MARISA}/marisa/scoped-array.h \
  $${PATH_MARISA}/marisa/key.h \
  $${PATH_MARISA}/marisa/keyset.h \
  $${PATH_MARISA}/marisa/query.h \
  $${PATH_MARISA}/marisa/agent.h \
  $${PATH_MARISA}/marisa/stdio.h \
  $${PATH_MARISA}/marisa/iostream.h \
  $${PATH_MARISA}/marisa/trie.h


INSTALLS += header_single header_files target
