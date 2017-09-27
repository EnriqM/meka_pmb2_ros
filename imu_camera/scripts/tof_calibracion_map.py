#!/usr/bin/env python

'''
Created on 21 ago. 2017

@author: Enrique Ortega

Script creado para guardar la calibracion de la TOF respecto al mapa
y posteriormente cargarlo.


'''
import os, rospkg, rospy, subprocess
import pickle
import tf



class calibracion(object):
    '''
    Clase que alberga todas las funciones utilizadas en el script
    '''
    def __init__(self):
        '''
        Constructor
        '''
        self.fixedname = 'map'
        movename = 'SwissRanger/base'
        self.fixedframe = self.fixedname
        self.moveframe = movename
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.id_filename = 'calibracion.txt'
        self.rate = rospy.Rate(10.0)
        

    def transformaciones_tf2(self, tiempo):
        self.listener.waitForTransform(self.moveframe, self.fixedframe, tiempo, rospy.Duration(1.0))
        (trans,rot) = self.listener.lookupTransform(self.moveframe, self.fixedframe, tiempo)# En lugar de now usabamos: rospy.Time(0)
        rotation = [-rot[0], -rot[1], -rot[2], rot[3]]
        trans  = tf.transformations.translation_matrix(trans)
        rot =  tf.transformations.quaternion_matrix(rot)
        transform = tf.transformations.concatenate_matrices(trans, rot)
        inversed_transform = tf.transformations.inverse_matrix(transform)
        transf = tf.transformations.translation_from_matrix(inversed_transform)
        (trans,rot) = self.listener.lookupTransform(self.moveframe, self.fixedframe, tiempo)# En lugar de now usabamos: rospy.Time(0)
        return (transf, rotation)
    
    def graba_posicion(self, filename):
        now = rospy.Time(0)
        (trans, rot) = self.transformaciones_tf2(now)       
        vector = [trans[0],trans[1],trans[2], rot[0], rot[1], rot[2], rot[3]]
        #Save transform to file
        file_path = os.path.join(rospkg.RosPack().get_path('imu_camera'), 'data', filename)
        fileObject = open(file_path, 'wb')
        pickle.dump(vector, fileObject) #guardar datos en fichero
        fileObject.close()
        
    def readPersonData(self, filename):
        file_path = os.path.join(rospkg.RosPack().get_path('imu_camera'), 'data', filename)#ruta relativa a paquete
        file_object = open(file_path, 'rb')
        person_data = pickle.load(file_object) #cargar datos de vuelta
        file_object.close()
        return person_data
    
    def elimina_nodos(self):
        p1 = subprocess.Popen(['rosnode', 'kill', '/SwissRanger_amcl', 
                               '/bl_laser', '/swiss_tf', '/laser_scan_matcher_node', 
                               '/pointcloud_to_laserscan' ], stdout=subprocess.PIPE)
        output = p1.communicate()[0]
        print output
        
    
    def emite_tf(self, filename):
        '''
        El frame al que se referencia ahora solo es Swisranger, ya que no hara falta lo de pointcloud to laser.
        '''
        while not rospy.is_shutdown():
            try:
                self.br.sendTransform((filename[0], filename[1], filename[2]),
                     (filename[3], filename[4], filename[5], filename[6]),
                     rospy.Time.now(),
                     'SwissRanger',
                     self.fixedname)
                self.rate.sleep()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException):
                pass
                

if __name__ == '__main__':
    rospy.init_node('tof_calibration_map')
    modo_escritura = rospy.get_param('~modo_escritura')
    clase = calibracion()
    if modo_escritura == True:
        clase.graba_posicion(clase.id_filename)
        clase.elimina_nodos()
        
    lectura = clase.readPersonData(clase.id_filename)
    print lectura 
    clase.emite_tf(lectura)