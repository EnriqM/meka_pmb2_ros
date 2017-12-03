#!/usr/bin/env python

'''
Created on 2 sept. 2017

@author: Enrique Ortega

Descripcion: GUI o Interfaz grafica creada para gestionar funcionalidades
   creadas para el TFG de Enrique Ortega, basado en la implementacion del
    robot Meka y el robot PMB2, junto con la camara Kinect.

'''

import rosnode #, rospy, std_msgs.msg, signal, threading, datetime
import subprocess

import os
from gi.repository import Gtk #importar libreria grafica
from termcolor import colored
import rospkg
import rosgraph
from time import sleep
#from wx.lib.masked.maskededit import control
#import random


class Platform_GUI(Gtk.Window):
    
    processes = []
    
    def __init__(self):
        #Gtk.Window.__init__(self, title="FileChooser Example")
        self.builder = Gtk.Builder()
        self.builder.add_from_file("gui_final.glade")#Cargara el archivo glade
        
        '''
        self.handlers = {"terminar_aplicacion": Gtk.main_quit,
                         "evento_conversion": self.convertir,
                         "evento_frase": self.mostrar_frase}
        '''
        self.handlers = {"terminar_aplicacion": Gtk.main_quit,
                         "evento_arranque": self.arranque,
                         "evento_Meka": self.meka,
                         "evento_SLAM": self.slam,
                         "evento_guardamap": self.save_map,
                         "evento_calibracion": self.calibracion_Tof,
                         "evento_terminacalib": self.end_calibration,
                         "evento_cargamap": self.load_map,
                         "evento_NAV": self.start_navigation,
                         "evento_final": self.start_smach}
        self.builder.connect_signals(self.handlers)#Conectar senyales
        self.window = self.builder.get_object("ventana_principal")
        self.window.show_all()
        
        self.map_name_entry = self.builder.get_object("map_name_entry") #Entrada de texto con el mapa
        self.combo_robot = self.builder.get_object("combo_robot") # Seleccion de modo del robot Meka
        self.nombre_mapa = self.builder.get_object("nombre_mapa") # Etiqueta de texto con el nombre del mapa
        self.info_meka = self.builder.get_object("info_meka")
        self.info_slam = self.builder.get_object("info_slam")
        self.map_path_files = os.path.join(rospkg.RosPack().get_path('pmb2_grav_2dnav'), 'maps', 'gui_mapas') + '/'
        self.own_path = os.path.join(rospkg.RosPack().get_path('platform_bringup'), 'scripts') + '/'
        self.map_name = ""
        
        
        
    def infoMessage(self):
        #Es solo texto con colores y negrita en VENTANA de COMANDOS
        print '\033[1m' + colored('Remember: do not close using CTRL+C, use "close" button instead!', 'red') + '\033[0m' #print red, bold
        print '\033[1m' + colored("Introduce person's name without spaces", 'red') + '\033[0m'
        
    def launchTerminal(self, command, hidden = False): #usar comillas dobles en command
        #hidden si es False sale la ventana, y si es True se minimiza. Lo hace con XTERM
        if hidden:
            process = subprocess.Popen('xterm -bg grey12 -fa "Monospace" -fs 11 -iconic -e "' + command + '"', shell=True, preexec_fn = os.setsid)
        else:
            process = subprocess.Popen('xterm -bg grey12 -fa "Monospace" -fs 11 -e "' + command + '"', shell=True, preexec_fn = os.setsid)
        self.processes.append(process)
        return process
        
    def testNodePing(self, node_name):
        #Simplemente una funcion que devuelve bool de nodo si activo o no. SI NECESITA PALITO.
        master = rosgraph.Master('/rosnode')
        node_api = rosnode.get_api_uri(master, node_name)
        node_is_active = False
        if node_api:
            node_is_active = rosnode.rosnode_ping(node_name, 1, verbose=False) #1: max ping request count
        return node_is_active
        
    def arranque(self, button):
        
        command_pal_killer = "rosnode kill  /compressed_map_publisher /map_configuration_server /map_setup /move_base /pal_navigation_sm /pal_vo_server /pose_saver /map_server /amcl"
        
        node_is_active = self.testNodePing('/move_base')
        if node_is_active == True:
            print 'PAL Navigation is on. Don not start another or stop it first'
            #self.launchTerminal(command_pal_killer, hidden = False)
            sleep(1)           
        else:
            print 'PAL Navigation is off. Synchronising clocks.'
        self.launchTerminal("python "+ self.own_path + "ssh_sudo_meka.py", True)
        self.launchTerminal("roslaunch platform_bringup basic_platform.launch", True)
	print "Platform prepared."
        #self.launchTerminal("python "+ self.own_path + "ssh_pmb2.py", True)
        
        
    def meka(self, button):
        sim_active = self.testNodePing('/simMeka')
        control_active = self.testNodePing('/controlMeka')
        if self.combo_robot.get_active_text() == "Simulador":
            
            if sim_active == True:
                self.info_meka.set_text('Meka simulator already up.')
            elif control_active == True:
                self.info_meka.set_text('Do not launch simulator if control is active!')
            else:
                self.launchTerminal("roslaunch platform_bringup simMeka.launch", True)
        else:
            if control_active == True:
                self.info_meka.set_text('Meka control is already up.')
            elif sim_active == True:
                self.info_meka.set_text('Closing simulator first. Please, wait.')
                self.launchTerminal("roslaunch platform_bringup controlMeka.launch", True)
            else:
                self.info_meka.set_text("DO NOT CLOSE controlMeka command window")
                self.launchTerminal("roslaunch platform_bringup controlMeka.launch", True)

                
        
        
    def slam(self, button): 
        node_is_active = self.testNodePing('/slam_gmapping')
        if node_is_active == True:
            print "SLAM already active. Please, save the map"
        else:
            self.launchTerminal("roslaunch platform_bringup justPMB2slam.launch", False)
        
            
    def save_map(self,button):   
        slam_is_active = self.testNodePing('/slam_gmapping')
        map_active = self.testNodePing('/map_server')
        
        if slam_is_active == True and map_active == False:
            if str(self.map_name_entry.get_text()) == '':
                self.info_slam.set_text("Por favor, introduzca el nombre de un mapa")
            else:
                self.info_slam.set_text("Guardando mapa. Por favor, espere.")
                self.launchTerminal("rosrun map_server map_saver -f "
                                    + self.map_path_files + str(self.map_name_entry.get_text()), False)
                sleep(10)
                self.launchTerminal("rosnode kill /slam_gmapping", False)
        elif slam_is_active == False:
            self.info_slam.set_text("Por favor, ejecute SLAM primero")
        #slam_gmapping move_base rviz
        print str(self.map_name_entry.get_text())
        
            
    def calibracion_Tof(self,button):
        node_is_active = self.testNodePing('/SwissRanger_amcl')
        if node_is_active == True:
            print "Calibration already active. Press the other button if it is ended"
        else:
            self.launchTerminal("roslaunch imu_camera laser_scan.launch", False)
        
    def end_calibration(self,button):
        calib = self.testNodePing('/SwissRanger_amcl')
        calib_end = self.testNodePing('/tof_calibration_node')
        if calib == True and calib_end == False:
            self.launchTerminal("roslaunch imu_camera start_tof.launch bool_escritura:=True", True)
        elif calib == False:
            print "Start calibration first"
        else:
            print "Wait until calibration is completely ended."
        
    def load_map(self,button):
        ruta = self.carga_archivo()
        if ruta == 'Cancelled' or ruta == None:
            self.nombre_mapa.set_text("Escoja un archivo valido")
        else:
            self.map_name = ruta
            lista = ruta.split("/")
            just_name = ''.join(lista[-1:])
            self.nombre_mapa.set_text(str(just_name))   
            print self.map_name
            print str(just_name)      
        
        
    def start_navigation(self,button):       
        node_is_active = self.testNodePing('/move_base')
        if node_is_active == True:
            print 'Navigation is on. Powering it off'
            self.launchTerminal("rosnode kill /move_base", hidden = False)
            sleep(1)
        else:
            if self.map_name == "":
                print "Using default map"
                self.launchTerminal("roslaunch platform_bringup justPMB2nav.launch", True)
            else:
                self.launchTerminal("roslaunch platform_bringup justPMB2nav.launch map_file:=" + self.map_name, True)
        
        
    def start_smach(self,button):
        node_is_active = self.testNodePing('/server_arm_follower') 
        smach_act = self.testNodePing('/wait_and_navigate_smach')
        if node_is_active == False and smach_act == False:#nav_to_people)/launch/server_meka.launch
            self.launchTerminal("roslaunch nav_to_people server_meka.launch", False)
	    self.launchTerminal("roslaunch kinect_skeleton_ident test_fnn.launch", False)
            #self.launchTerminal("rosrun demo_smach final_demo_smach.py", False)
	    self.launchTerminal("rosrun demo_smach heibus.py", False)
        elif node_is_active == False:
            #self.launchTerminal("rosrun demo_smach final_demo_smach.py", False)
	    self.launchTerminal("rosrun demo_smach heibus.py", False)
        else:
            print "Ya iniciado."
        
        
            
    
        
    def carga_archivo(self):
        dialog = Gtk.FileChooserDialog("Please choose a file", None,#self, CON PONER NONE BASTA    
            Gtk.FileChooserAction.OPEN,
            (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
             Gtk.STOCK_OPEN, Gtk.ResponseType.OK))

        self.add_filters(dialog)

        response = dialog.run()
        if response == Gtk.ResponseType.OK:
            print "Open clicked"
            print "File selected: " + dialog.get_filename()
            path = str(dialog.get_filename())
        elif response == Gtk.ResponseType.CANCEL:
            print "Cancel clicked"
            path = "Cancelled"

        dialog.destroy()
        return path

    def add_filters(self, dialog):
        filter_text = Gtk.FileFilter()
        filter_text.set_name("Text files")
        filter_text.add_mime_type("text/plain")
        dialog.add_filter(filter_text)

        filter_py = Gtk.FileFilter()
        filter_py.set_name("Python files")
        filter_py.add_mime_type("text/x-python")
        dialog.add_filter(filter_py)

        filter_any = Gtk.FileFilter()
        filter_any.set_name("Any files")
        filter_any.add_pattern("*")
        dialog.add_filter(filter_any)

if __name__ == '__main__':
    Platform_GUI()
    Gtk.main()
