"""Build a drivable hatchback from the already downloaded Gazebo Fuel mesh."""
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
SOURCE = Path.home() / '.gz/fuel/fuel.gazebosim.org/openrobotics/models/hatchback%20blue/5'
OUT = ROOT / 'models/hatchback_drivable'


def element(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child


def build():
    meshes = OUT / 'meshes'
    meshes.mkdir(parents=True, exist_ok=True)
    vertices, normals, uvs, faces = [], [], [], {}
    group = ''
    for line in (SOURCE / 'meshes/hatchback.obj').read_text().splitlines():
        s = line.split()
        if not s:
            continue
        if s[0] in ('v', 'vn'):
            x, y, z = map(float, s[1:4])
            transformed = (-y, x, z)
            if s[0] == 'v':
                vertices.append(tuple(a * 0.0254 for a in transformed))
            else:
                normals.append(transformed)
        elif s[0] == 'vt':
            uvs.append(line)
        elif s[0] == 'g':
            group = s[1]
        elif s[0] == 'f':
            idx = [int(a.split('/')[0])-1 for a in s[1:]]
            if group == 'Hatchback':
                name = 'body'
            else:
                center = [sum(vertices[i][j] for i in idx)/len(idx) for j in range(3)]
                name = ('front' if center[0] > 0 else 'rear') + ('_left' if center[1] > 0 else '_right')
            faces.setdefault(name, []).append((line.rstrip(), idx))
    centers = {}
    for name, entries in faces.items():
        used = set(i for _, indices in entries for i in indices)
        bounds = [(min(vertices[i][j] for i in used), max(vertices[i][j] for i in used)) for j in range(3)]
        center = (0, 0, 0) if name == 'body' else tuple((a+b)/2 for a, b in bounds)
        centers[name] = center
        contents = ['mtllib hatchback.mtl']
        contents += ['v ' + ' '.join(f'{a-b:.9f}' for a,b in zip(v,center)) for v in vertices]
        contents += uvs
        contents += ['vn ' + ' '.join(map(str,n)) for n in normals]
        contents += [f'g {name}', 'usemtl ' + ('Hatchback' if name == 'body' else 'Wheels3')]
        contents += [line for line,_ in entries]
        (meshes / f'{name}.obj').write_text('\n'.join(contents)+'\n')
    # Keep the original body texture; use the downloaded wheel texture if present.
    textures = OUT / 'materials/textures'
    textures.mkdir(parents=True, exist_ok=True)
    shutil.copy2(SOURCE / 'materials/textures/hatchback.png', textures / 'hatchback.png')
    wheel_texture = SOURCE.parent.parent / 'hatchback/3/materials/textures/wheels3.png'
    if wheel_texture.exists():
        shutil.copy2(wheel_texture, textures / 'wheels3.png')
    (meshes / 'hatchback.mtl').write_text(
        'newmtl Hatchback\nKa 0.6 0.6 0.6\nKd 0.8 0.8 0.8\n'
        'map_Kd ../materials/textures/hatchback.png\n'
        'newmtl Wheels3\nKa 0.15 0.15 0.15\nKd 0.25 0.25 0.25\n' +
        ('map_Kd ../materials/textures/wheels3.png\n' if wheel_texture.exists() else ''))

    sdf = ET.Element('sdf', version='1.9')
    model = element(sdf, 'model', name='hatchback_blue')
    element(model, 'static', 'false')
    element(model, 'self_collide', 'false')

    def inertia(link, mass, xyz, center='0 0 0 0 0 0'):
        inertial = element(link, 'inertial')
        element(inertial, 'pose', center)
        element(inertial, 'mass', mass)
        tensor = element(inertial, 'inertia')
        for key, val in zip(('ixx','iyy','izz'), xyz):
            element(tensor, key, val)
        for key in ('ixy','ixz','iyz'):
            element(tensor, key, 0)

    def mesh_visual(link, name):
        visual = element(link, 'visual', name=name+'_visual')
        mesh = element(element(visual, 'geometry'), 'mesh')
        element(mesh, 'uri', f'meshes/{name}.obj')

    chassis = element(model, 'link', name='chassis')
    inertia(chassis, 900, (320, 1350, 1500), '0.25 0 0.65 0 0 0')
    mesh_visual(chassis, 'body')
    for name, pose, size in [('body_collision','0.3435 0 0.62 0 0 0','4.0 1.86 0.85'),
                              ('cabin_collision','0.0 0 1.20 0 0 0','2.1 1.65 0.65')]:
        collision = element(chassis, 'collision', name=name)
        element(collision, 'pose', pose)
        element(element(element(collision, 'geometry'), 'box'), 'size', size)

    def joint(name, parent, child, axis, steering=False):
        j = element(model, 'joint', name=name, type='revolute')
        element(j,'parent',parent)
        element(j,'child',child)
        a = element(j,'axis')
        element(a,'xyz',axis)
        limit = element(a,'limit')
        element(limit,'lower',-0.60 if steering else -1e16)
        element(limit,'upper',0.60 if steering else 1e16)
        element(limit,'effort',2000)
        element(limit,'velocity',3 if steering else 80)
        element(element(a,'dynamics'),'damping',0.05)

    for name in ('front_left','front_right','rear_left','rear_right'):
        center = centers[name]
        pose = ' '.join(map(str, center)) + ' 0 0 0'
        parent = 'chassis'
        if name.startswith('front'):
            knuckle = element(model,'link',name=name+'_knuckle')
            element(knuckle,'pose',pose)
            inertia(knuckle, 2, (0.01,0.01,0.01))
            joint(name+'_steer','chassis',name+'_knuckle','0 0 1',True)
            parent = name+'_knuckle'
        link = element(model,'link',name=name+'_wheel')
        element(link,'pose',pose)
        inertia(link, 15, (0.50,0.77,0.50))
        mesh_visual(link, name)
        collision = element(link,'collision',name='tire')
        element(collision,'pose','0 0 0 1.57079632679 0 0')
        cylinder = element(element(collision,'geometry'),'cylinder')
        element(cylinder,'radius',0.3205)
        element(cylinder,'length',0.28)
        ode = element(element(element(collision,'surface'),'friction'),'ode')
        element(ode,'mu',1.0)
        element(ode,'mu2',1.0)
        joint(name+'_spin',parent,name+'_wheel','0 1 0')

    plugin = element(model,'plugin',filename='gz-sim-ackermann-steering-system',
                     name='gz::sim::systems::AckermannSteering')
    for side in ('left','right'):
        for axle in ('front','rear'):
            element(plugin,side+'_joint',axle+'_'+side+'_spin')
        element(plugin,side+'_steering_joint','front_'+side+'_steer')
    for name,value in {'wheel_separation':1.57,'kingpin_width':1.57,'wheel_base':2.5165,
                       'wheel_radius':0.3205,'steering_limit':0.5,'steer_p_gain':8,
                       'min_velocity':-2,'max_velocity':2,'min_acceleration':-4,
                       'max_acceleration':4}.items():
        element(plugin,name,value)
    ET.indent(sdf)
    ET.ElementTree(sdf).write(OUT/'model.sdf',encoding='unicode',xml_declaration=True)
    (OUT/'model.config').write_text('<model><name>Drivable hatchback</name><version>1.0</version>'
                                  '<sdf version="1.9">model.sdf</sdf></model>')
    print(OUT/'model.sdf')
    print('Wheel centers:',centers)


if __name__ == '__main__':
    build()
