#! python3
######################################################################
#
#   This file is part of SimControl.
#
#   SimControl is free software: you can redistribute it and/or modify it under
#   the terms of the GNU General Public License as published by the Free
#   Software Foundation, either version 3 of the License, or (at your option)
#   any later version.
#
#   SimControl is distributed in the hope that it will be useful, but WITHOUT
#   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#   more details.
#
#   You should have received a copy of the GNU General Public License along
#   with SimControl.  If not, see <https://www.gnu.org/licenses/>.
#
######################################################################

# Run: python3 setup.py build_ext --inplace


from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

sources = [
        'datareceiver',
        'simcontroller',
        'simdispatcher',
        'simqueue',
        'taskdispatcher',
        'timedeltasvc',
        'timewatcher',
        'util',
        'variablepredictor',
        'vectorprocessor',
        ]

extensions = [ Extension( a, [ a+'.py' ] ) for a in sources ]
#        Extension( 'comp.py',   [ 'comp.pyx' ] ),

setup(
    name = '',
    ext_modules = cythonize( extensions )
)
