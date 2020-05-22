import numpy
import threading
import time
import roslib
try:
	roslib.load_manifest('msgs_to_cv2')
except:
	pass
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged as Battery
import termios
import sys, tty



#-- Lectura inmediata por teclado
def getch():
    def _getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    return _getch()

class ReactiveBebop:
	
	def print_help(self, h_type):
		if h_type==0: # Ayuda para menu
			print('\n\n\n\
[0] Modo manual \n\
[1] Seleccionar target (solo funciona en modo automatico)\n\
[2] Seleccionar velocidad [0.0 - 1.0]\n\
[3] Empezar/Parar grabacion\n\
[4] Mostrar % bateria\n\
[p] Parada de emergencia\n\
\n\n\n')

		elif h_type==1: # Ayuda para movimiento
			print('\n\n\n\
[ ] Despega\n\
[l] Aterrizar\n\
[w] Avanza\n\
[a] Desplaza izda\n\
[s] Retrocede\n\
[d] Desplaza dcha\n\
[8] Asciende\n\
[4] Gira izda\n\
[6] Gira dcha\n\
[2] Desciende\n\
[5] Para\n\
[e] Exit. Modo auto\n\
[p] Parada de emergencia\n\
[h] Help\n\
\n\n\n')

	def __init__(self, target=None, tracks=[], current_track=None):
		self.target = target #id_track
		self.tracks = tracks
		self.current_track = current_track
		self.min_x = 500.0 #480.0
		self.max_x = 780.0 #800.0
		self.min_y = 180.0
		self.max_y = 540.0
		self.auto = True
		self.vel = 0.2
		self.min_height = 40.0
		self.max_height = 80.0
		self.record = False
		self.show_battery = False
		self.battery = 'unknown'
		self.aux1 = 0
		
		#-- Topics
		self.takeoff_pub = rospy.Publisher('/bebop/takeoff',Empty, queue_size=1)
		self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
		self.reset_pub = rospy.Publisher('/bebop/reset', Empty, queue_size=1)
		self.move_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
		self.video_pub = rospy.Publisher('/bebop/record', Bool, queue_size=1)
		self.battery_sub = rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged',Battery,self.battery_callback)

	def battery_callback(self, data):
		if not self.show_battery:
			self.battery = str(data).split(' ')[-1]

	def update_tracks(self, confirmed_tracks):
		self.tracks=confirmed_tracks
	
	def update_target(self, target):
		self.target=target
		
	def move(self, moves=[]):
		msg=Twist()
		for move in moves:
			if move==' ': # despegar
				self.takeoff_pub.publish(Empty())
			elif move=='l': # aterrizar
				self.land_pub.publish(Empty())
			elif move=='w': # avanzar
				msg.linear.x = self.vel
			elif move=='a': # desplazar a la izda
				msg.linear.y = self.vel	
			elif move=='s': # retroceder
				msg.linear.x = -self.vel
			elif move=='d': # desplazar a la derecha
				msg.linear.y = -self.vel
			elif move=='8': # ascender
				msg.linear.z = self.vel
			elif move=='4': # rotar izda
				msg.angular.z = self.vel
			elif move=='6': # rotar dcha
				msg.angular.z = -self.vel
			elif move=='2': # descender
				msg.linear.z = -self.vel
			elif move=='5': # para
				msg.linear.z = 0
				msg.angular.z = 0
				msg.linear.x = 0
				msg.linear.y = 0
			elif move=='e': # cambiar de modo
				self.auto=True
			elif move=='p': # parada de emergencia
				self.reset_pub.publish(Empty())
			elif move=='h': # ayuda
				self.print_help(1)
		
		# Si no mandamos un mensaje cada 0.1s
		# el dron detecta que hay un error
		# y se mantiene estatico
		self.move_pub.publish(msg)
	
	def follow_target(self):
		time.sleep(2) # Damos tiempo a que carguen el resto de hebras
		while(True):
			moves=[]
			if self.auto and self.current_track != None:
				if self.current_track.state == 2:
					centroid = self.current_track.centroid_coor
					bbox = self.current_track.bbox
					h = bbox[2] # 0- x coord, 1- y coord, 2- height, 3- width
					
					if centroid[0] < self.min_x:
						moves.append('4')
					elif centroid[0] > self.max_x:
						moves.append('6')
					"""
					if centroid[1] < self.min_y:
						moves.append('8')
					elif centroid[1] > self.max_y:
						moves.append('2')	
					"""
					if h < self.min_height:
						moves.append('w')
					elif h > self.max_height:
						moves.append('s')
				
			self.move(moves)
		
	def menu(self):
		time.sleep(2) # Damos tiempo a que carguen el resto de hebras
		while(True):
			time.sleep(0.1)
			self.print_help(0) # Imprime ayuda menu
			option = getch()
			
			while(option!='0' and option!='1' and option!='2' and option!='3' and option!='4' and option!='p'):
				option = getch()
				
			if option=='0': # modo manual
				self.auto = False
				c=''
				self.print_help(1) # Imprime ayuda movimiento
				
				while(c!='e'):
					c=getch()
					try:
						self.move([c]) # Realiza movimiento 
					except:
						print('Tecla no valida')
						self.move('h')
					
			elif option=='1': # Seleccion de target
				ids = [t.id for t in self.tracks]
				print('\'-1\'Exit.\n Select target: ')
				try:
					new_target = input()
				except:
					new_target = -2
				
				while( (not new_target in ids) and new_target != -1):
					print('Bad Target. Select one of this: ')
					ids = [t.id for t in self.tracks]
					print(ids)
					print('\'-1\'Exit.\n Select target: ')
					try:
						new_target = input()
					except:
						new_target = -2
				
				if new_target != -1:
					self.target = new_target
					self.current_track = [tr for tr in self.tracks if tr.id==new_target][0]
					print('Target Updated')
					print(self.current_track.bbox)
				
			elif option=='2': # Seleccion de velocidad
				print('Velocidad actual: '+str(self.vel)+'\nIndique nueva velocidad [0.0 - 1.0]: ')
				try:
					v = input()
					if v>=0 and v<=1:
						self.vel = v
						print('Velocidad actualizada')
					else:
						print('Velocidad fuera de los limites')
				except:
					print('Error en la entrada de velocidad')
				
			elif option=='3': # Empezar/Parar grabacion
				if not self.record:
					self.record = True
					self.video_pub.publish(True)
					print('Ha comenzado la grabacion\n')
				else:
					self.record = False
					self.video_pub.publish(False)
					print('Se ha detenido la grabacion\n')
				
			elif option=='4': # Mostrar % bateria
				self.show_battery = True
				print('Bateria: '+self.battery+'%')
				self.show_battery = False
				
			elif option=='p': # Parada de emergencia
				self.move([option])
	
	def start(self):
		self.b = threading.Thread(target=self.follow_target, args=())
		self.b.daemon = True
		self.b.start()
		self.c = threading.Thread(target=self.menu, args=())
		self.c.daemon = True
		self.c.start()





















