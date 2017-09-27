'''
Created on 11 jul. 2017

@Author: Enrique Ortega
'''

import tf
import rospy
import math


def calculo_angulo_tf(msg, movename, newname):
    num_per=int(movename[-1:]) #Esto es porque el vector de vectores se referencia al numero
    #La siguiente linea se puede descomentar para depurar
    #rospy.loginfo("msg in X is: %d" % (msg.coords[num_per].X_Coord))
    #rospy.loginfo("msg in Z is: %d" % (msg.coords[num_per].Z_Coord))
    angulo_rad= math.atan2(msg.coords[num_per].X_Coord, msg.coords[num_per].Z_Coord)
    br = tf.TransformBroadcaster()
    br.sendTransform((0.0, 0.0, 0.0),
                     tf.transformations.quaternion_from_euler(0, 0, angulo_rad+3.14159265),
                     rospy.Time.now(),
                     newname,
                     movename)
    #Se suma PI radianes porque el TF originalmente se encuentra 180 girado, en la misma direccion de la camara
    
    
def transformaciones_tf(move, fix, tiempo, listener):
    (trans,rot) = listener.lookupTransform(move, fix, tiempo)# En lugar de now usabamos: rospy.Time(0)
    trans  = tf.transformations.translation_matrix(trans)
    rot =  tf.transformations.quaternion_matrix(rot)
    transform = tf.transformations.concatenate_matrices(trans, rot)
    inversed_transform = tf.transformations.inverse_matrix(transform)
    #rotation = tf.transformations.rotation_from_matrix(inversed_transform)
    return tf.transformations.translation_from_matrix(inversed_transform)

def transformaciones_tf2(move, fix, tiempo, listener):
    (trans,rot) = listener.lookupTransform(move, fix, tiempo)# En lugar de now usabamos: rospy.Time(0)
    trans  = tf.transformations.translation_matrix(trans)
    rot =  tf.transformations.quaternion_matrix(rot)
    transform = tf.transformations.concatenate_matrices(trans, rot)
    inversed_transform = tf.transformations.inverse_matrix(transform)
    rotation = tf.transformations.rotation_from_matrix(inversed_transform)
    transf = tf.transformations.translation_from_matrix(inversed_transform)
    return (rotation, transf)
class transforaciones_tf_clase(object):
    def transformaciones_base(self, move, fix, tiempo, listener):
        (trans,rot) = listener.lookupTransform(move, fix, tiempo)# En lugar de now usabamos: rospy.Time(0)
        trans  = tf.transformations.translation_matrix(trans)
        rot =  tf.transformations.quaternion_matrix(rot)
        transform = tf.transformations.concatenate_matrices(trans, rot)
        inversed_transform = tf.transformations.inverse_matrix(transform)
        self.rotacion = tf.transformations.rotation_from_matrix(inversed_transform)
        self.translacion = tf.transformations.translation_from_matrix(inversed_transform)
        
    def transformaciones_tf(self, move, fix, tiempo, listener):
        self.transformaciones_base(move, fix, tiempo, listener)
        return self.translacion
    def transformaciones_tf2(self, move, fix, tiempo, listener):
        self.transformaciones_base(move, fix, tiempo, listener)
        return (self.rotacion, self.translacion)
    def yaw(self, move, fix, tiempo, listener, vuelta):
        final_trans = self.transformaciones_tf(move, fix, tiempo, listener)
        if vuelta:
            return (math.radians(180.0) + math.atan2(final_trans[1], final_trans[0]))
        else:
            return math.atan2(final_trans[1], final_trans[0])
    def trans_human_camera(self, move, tiempo, listener):
        #El fixedframe es /map con toda seguridad, por lo que no se incluye como parametro de entrada.
        #No se devuelve en negativo para evitar confusiones
        (rot, tran) = self.transformaciones_tf2(move, "/map", tiempo, listener)
        Z = tran[2]
        euler = tf.transformations.euler_from_quaternion(rot)
        roll = euler[0]
        pitch = euler[1]
        return (roll, pitch, Z)

def yaw(move, fix, tiempo, listener, vuelta):
    #Si vuelta es True, se le anyaden 180.0 grados para que este orientado hacia la camara
    final_trans = transformaciones_tf(move, fix, tiempo, listener)
    if vuelta:
        return (math.radians(180.0) + math.atan2(final_trans[1], final_trans[0]))
    else:
        return math.atan2(final_trans[1], final_trans[0])
    
    
def trans_human_camera(move, tiempo, listener):
    #El fixedframe es /map con toda seguridad, por lo que no se incluye como parametro de entrada.
    #No se devuelve en negativo para evitar confusiones
    (rot, tran) = transformaciones_tf2(move, "/map", tiempo, listener)
    Z = tran[2]
    euler = tf.transformations.euler_from_quaternion(rot)
    roll = euler[0]
    pitch = euler[1]
    return (roll, pitch, Z)
    
    
    
    
def velocidad_humano(moveframe, fixedframe, tiempo, duracion, listener):
    #La duracion va en segundos
    (linear, angular)=listener.lookupTwist(moveframe, fixedframe, tiempo, rospy.Duration(duracion)) #para calcular la velocidad de la persona          
    return math.sqrt(math.pow(linear[0], 2) + math.pow(linear[1], 2))
    
     
