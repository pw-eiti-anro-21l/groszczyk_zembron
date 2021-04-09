
import os
import PyKDL
import mathutils

filepath='parametryDH.txt'
pathYaml='urdf/UrdfParameters.yaml'

#parametry elementów
base_y=0.2
element1_param=[0.2, 0.2, 1.6]
element2_param=[0.2, 0.2, 0.2]
element3_param=[0.2, 0.2, 0.6]
element4_param=[]
element_param=[element1_param, element2_param, element3_param]
data=[['r1'], ['r2'], ['r3']]

def readParameters(plik):
	i=0
	if os.path.isfile(plik):
		with open(plik, "r") as f:
			for line in f:
				line=line.strip('\n') 
				line=line.split(" ")
				data[i].append(line)
				i+=1	
		f.close()		
		
	else:
		print("Błąd odczytu danych!")



def DHtransform(plikYaml):
	#otwiera plik do zapisu

	if os.path.isfile(plikYaml):
		with open(plikYaml, "w") as f:
			i=0
			for dat in data:
				name=dat[0]
				a=float(dat[1][0])
				d=float(dat[1][1])
				alpha=float(dat[1][2])
				theta=float(dat[1][3])
				box_x= element_param[i][0]
				box_y= element_param[i][1]
				box_z= element_param[i][2]


 
				trans_z= mathutils.Matrix.Translation((0, 0, d))
				rot_z = mathutils.Matrix.Rotation(theta, 4, 'Z')
				trans_x = mathutils.Matrix.Translation((a, 0, 0))
				rot_x =mathutils.Matrix.Rotation(alpha,4,  'X')

				# przemnożenie macierzy
				m = trans_x @ rot_x @ rot_z @trans_z

				rpy = m.to_euler()
				xyz = m.to_translation()
				if i==1:
					alphaElement4=-rpy[0]
				f.write(name + ":\n")
				if(name == "r1"):
					f.write(" xyz: "+str(xyz[0])+" "+str(xyz[1])+" "+str(xyz[2]+base_y+(box_z/2))+"\n")
				if(name == "r2"):
					f.write(" xyz: "+str(xyz[0])+" "+str(xyz[1])+" "+str(xyz[2]+base_y/2+element1_param[2]/2)+"\n")
				if(name == "r3"):
					f.write(" xyz: "+str(xyz[0])+" "+str(xyz[1])+" "+str(xyz[2])+"\n")
				f.write(" rpy: "+str(rpy[0])+' '+str(rpy[1])+' '+str(rpy[2])+'\n')
				f.write(" size: "+str(box_x)+' '+str(box_y)+' '+str(box_z)+'\n')
							
				i+=1
			f.write("r4" + ":\n")
			box_z=xyz[0]-box_x
			a_na_2=xyz[0]/2
			f.write(" element_xyz: "+str(0.0)+' '+str(0.0)+' '+str(a_na_2)+'\n')
			f.write(" size: "+str(0.2)+' '+str(box_y)+' '+str(box_z)+'\n')
			f.write(" rpy: "+str(0.0)+' '+str(alphaElement4)+' '+str(0.0)+'\n')
			f.write("limit" + ":\n")
			f.write(" lower: "+str(-1.57-alphaElement4)+'\n')
			f.write(" uper: "+str(1.57-alphaElement4)+'\n')

			f.close()




if __name__ == '__main__':
	readParameters(filepath)
	DHtransform(pathYaml)