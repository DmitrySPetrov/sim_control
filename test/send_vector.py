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

import struct
import socket
from math import sin, cos, pi, radians
from mathutils import Matrix, Quaternion, Vector, Euler
import datetime, threading

import numpy as np
#import random


# Со сложной матрицей перехода
#X0 = np.array( (
#		( 942164.40956471, ),
#		( 6710619.03304791, ),
#		( 218424.13997699, )
#	) )
#V0 = np.array( (
#		( -4736.34864762, ),
#		( 425.58696183, ),
#		( 6018.79573623, ),
#	) )

# С вращением вокруг оси Z
#X0 = np.array( (
#		( 918516.89111409, ),
#		( 6713860.77291344, ),
#		( 219524.324269, )
#	) )
#V0 = np.array( (
#		( -4746.92060108, ),
#		( 409.3949259, ),
#		( 6011.585938, )
#	) )

# Из матлабовской программы
X0 = np.array( (
		(   918525.454123263, ),
		(  6713859.601410753, ),
		(   219524.324269000, ),
	) )

V0 = np.array( (
		( -4742.990361055989, ),
		(   451.420385452605, ),
		(  6011.585938000000, ),
	) )

BETA = 0.8e-10
MU = 3.98576058e+14

t_sta0 = 1.47754736275709
OMGE = 7.2921158553e-5


class _calc_rva:

	MAX_H = 1.0

	def __init__( self, x=None, v=None ):
		self.beta = BETA
		self.mu = MU
		self._t_sta = t_sta0

#		self.x = np.array( [
#				[-269.214e+3], [4427.301e+3], [4959.75e+3],
#				[-7.067e+3], [-2.531e+3], [1.887e+3],
#				[ self.mu ], [ self.beta ],
#			] )
		_x = ( x, X0 )[ x == None ]
		_v = ( v, V0 )[ v == None ]
		self.x = np.vstack( (
				_x,
				_v,
				( ( self.mu, ), ),
				( ( self.beta, ), ),
			) )

		self.t_prev = 0.0

	@staticmethod
	def calc_f( t, y_j ):
		x = y_j[:3][:]
		v = y_j[3:6][:]
		mu = y_j[6][0]
		beta = y_j[7][0]
		res_x = v
		r3 = sum( _x*_x for _x in x ) ** 1.5
		res_v = ( - mu * x / r3 - beta * v )
		return np.vstack( ( res_x, res_v, np.array( [[0.0],[0.0]] ) ) )

	@staticmethod
	def calc_rk4_k( t, y, f, h ):
		k1 = f( t, y )
		k2 = f( t+h/2, y+h/2*k1 )
		k3 = f( t+h/2, y+h/2*k2 )
		k4 = f( t+h, y+h*k3 )
		return k1, k2, k3, k4

	def __call__( self, time: float ) -> tuple:
		_h = time - self.t_prev

		self.t_prev = time
		if _h > self.MAX_H:
			_h = 0.0

		self._t_sta += _h * OMGE

		y = self.x
		k1_4 = self.calc_rk4_k( time, y, self.calc_f, _h )
		dy = ( k1_4[0] + 2*k1_4[1] + 2*k1_4[2] + k1_4[3] )*_h/6
		y += dy
		self.x = y

#		if random.random() < _h/60.0:
#			self.x[1] += 0.05 * 13.3 / 6500.0
#		if random.random() < _h/60.0:
#			self.x[1] += 0.05 * 13.3 / 6500.0

		res = tuple( tuple( self.x[i+j*3][0] for i in range(3) ) for j in range(2) )
		s_out = '{:15.2f} ' + '{:10.6f} ' + '{:12.3f} '*3 + '{:12.6f} '*3
		print( s_out.format( time, self._t_sta, *( res[0] + res[1] ) ) )
		return res

	@property
	def t_sta( self ) -> float:
		return self._t_sta


class calc_rotation:

	def __init__( self, rv ):
		self.t_prev = 0.0
		self.rv = rv
		self.q_prev = None

	def __call__( self, time: float ) -> tuple:
		_h = time - self.t_prev
		# вычисляем новую ориентацию (ОСК)
		r = self.rv.x[:3]
		v = self.rv.x[3:6]
		eX = -v.reshape((3)) / np.linalg.norm( v )
		eZ = np.cross( r.reshape((3)), v.reshape((3)) )
		eZ /= np.linalg.norm( eZ )
		eY = np.cross( eZ, eX )
#		# вычисляем новую ориентацию, похожа на ОСК, но ось Y ровно вверх
#		eY = v.reshape((3)) / np.linalg.norm( v )
#		eZ = np.cross( r.reshape((3)), eY )
#		eZ /= np.linalg.norm( eZ )
#		eX = np.cross( eY, eZ )
		M_OSK = np.vstack( ( eX, eY, eZ ) )
		# вычисляем угловую скорость
		q_OSK = Matrix( M_OSK.tolist() ).to_quaternion()
		if self.q_prev and abs( _h ) > 1e-6:
			omega = np.array( q_OSK.cross( self.q_prev.inverted() )[1:] ) * 2/_h
		else:
			omega = np.array( [ 0.0, 0.0, 0.0 ] )
		self.q_prev = q_OSK
		self.t_prev = time
		print( ' '.join( '{:15.10f}'.format( a/pi*180 ) for a in q_OSK.to_euler( 'YXZ' )[:] ) )
		print( ' '.join( '{:8.5f}'.format( a/pi*180 ) for a in omega.tolist() ) )
		return ( tuple( M_OSK.reshape((9)).tolist() ), tuple( omega.tolist() ) )


calc_rva = _calc_rva()
calc_rvp = _calc_rva( 
		np.array( [ [-269.214e+3], [4427.301e+3], [4959.75e+3], ] ),
		np.array( [ [-7.067e+3], [-2.531e+3], [1.887e+3], ] ),
	)

calc_rotation_asc = calc_rotation( calc_rva )
calc_rotation_psc = calc_rotation( calc_rvp )


#def calc_rva( time: float ) -> tuple:
#	# радиус орбиты
#	R = 6800.0e3
#	# ускорение свободного падения
#	g = 9.81
#	# угловая скорость
#	w = ( g / R ) ** 0.5
#	# фаза
#	a = ( w * time ) % ( 2*pi )
#	# повороты: на a вокруг Z, на 51 градус вокруг X, на 120 градусов вокруг z
#	q1 = Quaternion( ( 0, 0, 1 ), a )
#	q2 = Quaternion( ( 1, 0, 0 ), radians( 51 ) )
#	q3 = Quaternion( ( 0, 0, -1 ), radians( 120 ) )
#	q = q3 * q2 * q1
#	# вычисляем вектор
#	r = Vector( ( R, 0, 0 ) )
#	r.rotate( q )
#	v = Vector( ( 0, R * w, 0 ) )
#	v.rotate( q )
#	# формируем результат
#	return ( r.to_tuple(), v.to_tuple() )

#def calc_rvp( time: float ) -> tuple:
#	# формируем результат
#	return ( ( 0.0, ) * 3, ) * 2
#



#def calc_rotation_asc( time: float ) -> tuple:
#	# матрица разворота
#	mat = ( 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 )
#	# угловые скорости
#	omga = ( 0.0, 0.0, 0.0 )
#	# результат
#	return ( mat, omga )
#
#def calc_rotation_psc( time: float ) -> tuple:
#	# матрица разворота
#	mat = ( 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 )
#	# угловые скорости
#	omga = ( 0.0, 0.0, 0.0 )
#	# результат
#	return ( mat, omga )
#

def send_vector( num: int, time: float ) -> None:
	rA = calc_rva( time )
	rotA = calc_rotation_asc( time )
	rP = calc_rvp( time )
	rotP = calc_rotation_psc( time )
	t_sta = calc_rva.t_sta
	buf = bytearray( 332 )
	data = \
		rA[0] + rA[1] + rotA[0] + rotA[1] \
		+ rP[0] + rP[1] + rotP[0] + rotP[1] \
		+ ( 0.0, ) * 3 + ( time, ) + ( t_sta, )
	struct.pack_into( '<BBHddddddddddddddddddddddddddddddddddddddddd', buf, 0, \
			num, 0, 0, *data )
	size = len( buf ) // 2
	r = struct.unpack( '<'+'H'*size, buf )
	sum = 0
	for i in range( size ):
		sum += r[i]
		while sum > 0xffff:
			sum = 1 + ( sum & 0xffff )
	struct.pack_into( '<H', buf, 2, sum )

	s = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
	s.sendto( buf, ( HOST, PORT ) )


class TimerCycler:
	def __init__( self, dt: float, start_time=None, T0_time=None ):
		self.__dt = dt
		self.__num = 0
		if start_time:
			self.__start_time = start_time
		else:
			self.__start_time = datetime.datetime.now()
		if T0_time:
			self.__cur_time = ( T0_time - self.__start_time ).total_seconds()
		else:
			self.__cur_time = ( datetime.datetime.now() - self.__start_time ).total_seconds()
		self.__old_time = datetime.datetime.now()

	def cycle( self ):
		new_time = datetime.datetime.now()
		self.__cur_time += ( new_time - self.__old_time ).total_seconds()
		self.__old_time = new_time
		print( self.__cur_time )
		send_vector( self.__num, self.__cur_time )
		self.__num = int( self.__num + 1 ) & 0xff
		# запускаем следующую итерацию
		threading.Timer( self.__dt, self.cycle ).start()

	start = cycle


if __name__ == '__main__':

	HOST = 'localhost'
	PORT = 5251

	#tc = TimerCycler( 0.5, start_time=datetime.datetime( 2000, 1, 1 ) ).start()
	tc = TimerCycler(
			0.05,
			start_time=datetime.datetime( 2000, 1, 1 ),
			T0_time=datetime.datetime( 2015, 8, 1, 12, 0, 0 )
#			T0_time=datetime.datetime( 2000, 1, 1, 12, 0, 0 )
		).start()

