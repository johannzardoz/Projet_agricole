import random

nb_arbre = int(input("nombre d'abres : "))

def debut():
    print('<?xml version="1.0"?>')
    print("<sdf version='1.4'>")
    print("    <world name='default'>")

def ciel_et_soleil(sunrise, clouds, sunset):
    #entre 0 et 24 sunrise
    #entre 0 et 24 clouds
    #entre 0 et 24 sunset
    print('	<scene>')
    print('		<sky>')
    print('			<sunrise>',sunrise,'</sunrise>' , sep='')
    print('			<clouds>',clouds,'</clouds>' , sep='')
    print('			<sunset>',sunset,'</sunset>' , sep='')
    print('		</sky>')
    print('	</scene>')
    print('	<include>')
    print('    		<uri>model://sun</uri>')
    print('		<pose>0.0 0.0 10.0  0 0 0</pose>')
    print('	</include>')

def create_arbre(number,pos_X,pos_Y):
    #creation du tronc
    print('		<link name="tronc_',number,'">', sep='')
    print('			<pose>',pos_X, pos_Y,'1.0  0 0 0</pose>')
    print('			<visual name="visual">')
    print('				<geometry>')
    print('					<box>')
    print('						<size>0.5 0.5 2.0</size>')
    print('					</box>')
    print('				</geometry>')
    print('				<material>')
    print('					<ambient>0.4 0.3 0.2 1</ambient>')
    print('					<diffuse>0.3 0.2 0.1 1</diffuse>')
    print('					<specular>0.1 0.1 0.1 1</specular>')
    print('					<emissive>0 0 0 0</emissive>')
    print('					<script>')
    print('                            			<uri>file://media/materials/scripts/gazebo.material</uri>')
    print('                            			<name>Gazebo/Red</name>')
    print('                        		</script>')
    print('				</material>')
    print('			</visual>')
    print('			<collision name="collision">')
    print('				<geometry>')
    print('					<box>')
    print('						<size>0.5 0.5 2.0</size>')
    print('					</box>')
    print('				</geometry>')
    print('			</collision>')
    print('		</link>')
    #creation du feuillage
    print('		<link name="feuillage_',number,'">', sep='')
    print('			<pose>',pos_X, pos_Y,'2.5  0 0 0</pose>')
    print('			<visual name="visual">')
    print('				<geometry>')
    print('					<sphere>')
    print('						<radius>6.0</radius>')
    print('					</sphere>')
    print('				</geometry>')
    print('				<material>')
    print('					<script>')
    print('                            			<uri>file://media/materials/scripts/gazebo.material</uri>')
    print('                            			<name>Gazebo/Green</name>')
    print('                        		</script>')
    print('				</material>')
    print('			</visual>')
    print('		</link>')


'''
for num in range (0,nb_arbre):
    x = random.randint(-40, 40)
    y = random.randint(-40, 40)
    create_arbre(num,x,y)
'''

ciel_et_soleil(5,10,10)