'''
Created on 19 aug. 2017

@author: Enrique Ortega
'''

import tf
import rospy
#import math
from sound_play.libsoundplay import SoundClient
from nav_to_people.srv import Permiso_Brazos, Permiso_BrazosResponse, Permiso_BrazosRequest
from robot_follower.srv import NumeroTF, NumeroTFResponse, NumeroTFRequest
    
def transformaciones_tf(move, fix, tiempo, listener):
    (trans,rot) = listener.lookupTransform(move, fix, tiempo)# En lugar de now usabamos: rospy.Time(0)
    trans  = tf.transformations.translation_matrix(trans)
    rot =  tf.transformations.quaternion_matrix(rot)
    transform = tf.transformations.concatenate_matrices(trans, rot)
    inversed_transform = tf.transformations.inverse_matrix(transform)
    #rotation = tf.transformations.rotation_from_matrix(inversed_transform)
    return tf.transformations.translation_from_matrix(inversed_transform)

def frame_contador(dic, counter):
    #Funcion para implementar el contador de personas.
    #Necesita el diccionario
    name_frame = dic.get(counter)
    if name_frame == None:
        return 'map'
    else:
        return name_frame


def habla(frase, espera):
    #Dice la frase mediante el nodo SoundPlay.
    #La espera se puede reconfigurar como parametro de entrada
    soundhandle = SoundClient()
    rospy.sleep(1.0)
    voice = 'voice_kal_diphone'  #Voz en ingles
    #voice = 'voice_el_diphone' #Voz en castellano.

    print 'Saying: %s' % frase
    print 'Voice: %s' % voice

    soundhandle.say(frase, voice)
    rospy.sleep(espera)
    

def cliente_kinect(activacion, objetivo, referencia):
    rospy.wait_for_service('permiso_kinect')
    set_kinect = rospy.ServiceProxy('permiso_kinect', NumeroTF)
    try:
        resp1 = set_kinect(activacion, objetivo, referencia)
        #resp1 = set_meka(flag, "mimica", "right", "right_hand_1")
        #rospy.loginfo('Respuesta = %s'%resp1.message)
        print resp1.message
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    

def cliente_meka(activacion, modo, brazo, frame_objetivo):
    '''
    Funcion para hacer uso del cliente que maneja la interaccion del Meka.
    Los parametros de entrada son los siguientes:
    activacion: Bool, debe ser True para activar o False para desactivar
    modo: String. "mimica" o "dar_mano"
    brazo: String. "left" o "right"
    frame_objetivo: String. Algunos ejemplos: "left_hand_1" o "right_hand_2"
    
    ----Devuelve un string con la respuesta que da el server que gestiona la interaccion---
    '''
    
    rospy.wait_for_service('permiso_meka_arms')
    set_meka = rospy.ServiceProxy('permiso_meka_arms', Permiso_Brazos)
    try:
        #resp1 = set_meka(flag, "dar_mano", "left", "left_hand_1")
        ##########resp1 = set_meka(flag, "mimica", "right", "right_hand_1")
        resp1 = set_meka(activacion, modo, brazo, frame_objetivo)
        #rospy.loginfo('Respuesta = %s'%resp1.message)
        return resp1.message
    except rospy.ServiceException as exc:
        return "Service did not process request: " + str(exc)
