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

__all__ = [
        'TaskDispatcher',
    ]


import time

class TaskDispatcher:

    class Task:
        def __init__( self, func, periodic, dtime ):
            self.__func = func
            self.__periodic = periodic
            self.__period = dtime
            self.__lastcall_time = time.time()
            return

        @property
        def periodic( self ):
            return self.__periodic

        @property
        def period( self ):
            return self.__period

        @property
        def lastcall_time( self ):
            return self.__lastcall_time

        @property
        def late( self ):
            l = time.time() - ( self.__lastcall_time + self.__period )
            if l > self.__period:
                l = l % self.__period
            return l

        def run( self, late ):
            self.__lastcall_time = time.time() - late
            self.__func()
            return

    def __init__( self, period ):
        self.__period = period
        self.__task = []
        self.__running = False
        return

    def run_once( self ):
        for t in self.__task[:]:
            late = t.late
            if late > 0:
                t.run( late )
                if not t.periodic:
                    self.__task.remove( t )
        return

    def run( self ):
        self.__running = True
        while self.__running:
            t = time.time()
            self.run_once()
            t = time.time() - t
            if t < self.__period:
                time.sleep( self.__period - t )
            else:
                #print( 'Lateness: {:10.5f}'.format( t - self.__period ) )
                pass
        return

    def add_periodic_task( self, func, period ):
        '''
        Добавить задачу func для вызова каждые period секунд
        '''
        self.__task += [ self.Task( func, True, period ) ]
        return

    def add_once_task( self, func, time ):
        '''
        Добавить задачу func для вызова через time секунд
        '''
        self.__task += [ self.Task( func, False, time ) ]
        return

    def stop( self ):
        self.__running = False


if __name__ == '__main__':

    class PrintOnce:
        def __init__( self ):
            return
        def __call__( self ):
            print( 'Once! {}'.format( time.time() ) )

    class PrintPeriodic:
        def __init__( self ):
            return
        def __call__( self ):
            print( 'Periodic! {}'.format( time.time() ) )

    d = TaskDispatcher( 0.01 )
    print( time.time() )
    d.add_periodic_task( PrintPeriodic(), 1.0 )
    d.add_once_task( PrintOnce(), 7.7 )
    d.add_once_task( PrintOnce(), 17.5 )
    d.add_once_task( PrintOnce(), 5.5 )

    d.run()

