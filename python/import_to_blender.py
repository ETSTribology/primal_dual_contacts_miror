import bpy
import json
import numpy
import mathutils
import math
import os
import bpy
import os
import bmesh

from bpy.props import StringProperty, BoolProperty
from bpy_extras.io_utils import ImportHelper
from bpy.types import Operator
from bpy import context


class OT_TestOpenFilebrowser(Operator, ImportHelper):

    bl_idname = "test.open_filebrowser"
    bl_label = "Import simulation"

    filter_glob: StringProperty(
        default='*.gltf',
        options={'HIDDEN'}
    )

    def execute(self, context):
        """Do something with the selected file(s)."""

        filenameWOex, extension = os.path.splitext(self.filepath)
        filenameBase = os.path.basename(filenameWOex)
        
        filename = filenameWOex + ".npz"
        sceneFile = self.filepath
        print("Loading dynamics...")
        inputFile = numpy.load(filename)

        print("Removing old meshes...")
        for key, val in inputFile.items():
            if key == "scene":
                continue
            meshName = filenameBase + "_" + key
            if meshName in bpy.data.meshes:
                bpy.data.meshes.remove(bpy.data.meshes[meshName])

        print("Importing scene...")
        bpy.ops.import_scene.gltf(filepath=sceneFile)

        times = inputFile["time"]
        times *= bpy.context.scene.render.fps
        times += 1
        
        emptyName = "empty_" + filenameBase
        if emptyName in bpy.data.objects:
            bpy.data.objects.remove( bpy.data.objects[emptyName] )
        empty = bpy.data.objects.new( "empty_" + filenameBase, None )
        bpy.context.scene.collection.objects.link( empty )

        print("Removing double vertices...")
        bm = bmesh.new()

        for m in bpy.data.meshes:
            bm.from_mesh(m)
            bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=0.0001)
            bm.to_mesh(m)
            m.update()
            bm.clear()
        bm.free()

        print("Inserting keyframes")
        currObj = 0
        for key, val in inputFile.items():
            currObj += 1
            if key == "scene" or key == "time":
                continue
            if currObj % 50 == 0 or currObj == len(inputFile.items()):
                print("\rProcessing object ({}/{})".format(currObj,
                                                           len(inputFile.items())), end='', flush=True)
            translations = val[1:, 0:3]
            rotations = val[1:, 3:7]

            scale = val[0, 0:3].flatten()
            scale[1], scale[2] = scale[2], scale[1]
            bpy.data.objects[key].scale = scale

            translations[:, [1, 2]] = translations[:, [2, 1]]
            translations[:, 1] = -translations[:, 1]

            rotations[:, [0, 1, 2, 3]] = rotations[:, [3, 0, 1, 2]]
            rotations[:, [2, 3]] = rotations[:, [3, 2]]
            rotations[:, 2] = -rotations[:, 2]
            timeOld, tOld, rOld = 1, mathutils.Vector(
                translations[0]), mathutils.Quaternion(rotations[0])

            obj = bpy.data.objects[key]
            currFrame = 1

            maxRec = min(times.shape[0],
                         translations.shape[0], rotations.shape[0])
            keyframes = numpy.zeros((math.floor(times[maxRec - 1]), 8))
            for time, t, r in zip(times, translations, rotations):
                tVec = mathutils.Vector(t)
                rQuat = mathutils.Quaternion(r)
                while (timeOld <= currFrame and time > currFrame):
                    line = (currFrame - timeOld) / (time - timeOld)
                    if line < 0.0:
                        line = 0.0
                    if line > 1.0:
                        line = 1.0

                    keyLoc = tOld.lerp(tVec, line)
                    keyQuat = rOld.slerp(rQuat, line)
                    keyframes[currFrame - 1, :] = [currFrame, keyLoc[0], keyLoc[1],
                                                   keyLoc[2], keyQuat[0], keyQuat[1], keyQuat[2], keyQuat[3]]
                    currFrame += 1
                timeOld, tOld, rOld = time, tVec, rQuat

            if not obj.animation_data:
                obj.animation_data_create()
            if not obj.animation_data.action:
                obj.animation_data.action = bpy.data.actions.new(
                    f"{obj.name}Action")
            obj.animation_data.action.fcurves.clear()
            for idx in range(3):
                fc = obj.animation_data.action.fcurves.new(
                    data_path="location", index=idx)
                fc.keyframe_points.add(keyframes.shape[0])
                fc.keyframe_points.foreach_set(
                    "co", keyframes[:, [0, idx+1]].flatten())
                for k in fc.keyframe_points:
                    # k.co[0] is the frame number
                     # k.co[1] is the keyed value
                    k.interpolation = 'CUBIC'
                    k.easing = 'EASE_IN'
            for idx in range(4):
                fc = obj.animation_data.action.fcurves.new(
                    data_path="rotation_quaternion", index=idx)
                fc.keyframe_points.add(keyframes.shape[0])
                fc.keyframe_points.foreach_set(
                    "co", keyframes[:, [0, idx+4]].flatten())
                for k in fc.keyframe_points:
                    # k.co[0] is the frame number
                     # k.co[1] is the keyed value
                    k.interpolation = 'CUBIC'
                    k.easing = 'EASE_IN'
            
            obj.name = filenameBase + "_" + key
            obj.data.name = filenameBase + "_" + key
            obj.parent = empty

        print("Loading finished")
        return {'FINISHED'}


def register():
    bpy.utils.register_class(OT_TestOpenFilebrowser)


def unregister():
    bpy.utils.unregister_class(OT_TestOpenFilebrowser)


register()

# test call
bpy.ops.test.open_filebrowser('INVOKE_DEFAULT')
