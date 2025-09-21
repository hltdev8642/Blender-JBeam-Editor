# Legacy-style JBeam exporter adapted from reference io_mesh_jbeam (GPL-derived portion)
# Integrated for feature parity: simple selection-based export of nodes, beams, and collision triangles.
# NOTE: This module is GPL-influenced; ensure overall distribution license compatibility.

import os
import bpy
import bmesh
from bpy import ops

from . import constants
from . import utils

class LEGACY_NGNode:
    __slots__ = ('id_', 'node_name', 'groups', 'x', 'y', 'z')
    def __init__(self, id_, node_name, groups, x, y, z):
        self.id_ = id_
        self.node_name = node_name
        self.groups = groups
        self.x = x
        self.y = y
        self.z = z

class JBEAM_EDITOR_OT_legacy_jbeam_export(bpy.types.Operator):
    bl_idname = 'jbeam_editor.legacy_jbeam_export'
    bl_label = 'Legacy Export (.jbeam)'
    bl_description = 'Export selected mesh objects to standalone .jbeam files (legacy simple mode)'
    bl_options = {'REGISTER'}

    export_scene: bpy.props.BoolProperty(
        name="Export Scene *.jbeam Meshes",
        description="Export all selectable mesh objects whose name ends with .jbeam",
        default=False,
    )

    def _gather_objects(self, context):
        if self.export_scene:
            out = []
            for o in context.selectable_objects:
                if o.type == 'MESH' and '.jbeam' in o.name:
                    out.append(o)
            return out
        return [o for o in context.selected_objects if o.type == 'MESH']

    def execute(self, context):
        export_objects = self._gather_objects(context)
        if not export_objects:
            self.report({'ERROR'}, 'No mesh objects selected for legacy export')
            return {'CANCELLED'}

        scene = context.scene
        base_dir = bpy.path.abspath('//jbeam_legacy_export/')
        os.makedirs(base_dir, exist_ok=True)

        for obj in export_objects:
            # Work on a temporary evaluated copy to freeze transforms
            depsgraph = context.evaluated_depsgraph_get()
            eval_obj = obj.evaluated_get(depsgraph)
            mesh = bpy.data.meshes.new_from_object(eval_obj, preserve_all_data_layers=True, depsgraph=depsgraph)

            bm = bmesh.new()
            bm.from_mesh(mesh)
            bm.verts.ensure_lookup_table()

            nodes = []
            for v in bm.verts:
                mv = mesh.vertices[v.index]
                nodes.append(LEGACY_NGNode(v.index, obj.data.get(constants.MESH_JBEAM_PART) or 'n', mv.groups,
                                           round(v.co.x, 3), round(v.co.y, 3), round(v.co.z, 3)))

            # Sorting similar to reference (z asc, x desc, y asc, then group)
            sorted_nodes = sorted(nodes, key=lambda n: n.z)
            sorted_nodes = sorted(sorted_nodes, key=lambda n: n.x, reverse=True)
            sorted_nodes = sorted(sorted_nodes, key=lambda n: n.y)
            sorted_nodes = sorted(sorted_nodes, key=lambda n: (len(n.groups) > 0, n.groups[0].group if len(n.groups) > 0 else 255))
            idx_map = {n.id_: i for i, n in enumerate(sorted_nodes)}

            if obj.name.endswith('.jbeam'):
                filename = obj.name
                part_name = obj.name[:-6]
            else:
                filename = obj.name + '.jbeam'
                part_name = obj.name

            filepath = os.path.join(base_dir, filename)
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write('{' + '\n')
                f.write(f'"{part_name}":{{\n')
                f.write('\t"information":{\n')
                f.write(f'\t\t"authors":"Blender JBeam Editor (Legacy Export)",\n')
                f.write(f'\t\t"name":"{part_name}",\n')
                f.write('\t},\n')
                f.write('\t"nodes":[\n\t\t["id","posX","posY","posZ"],\n')
                # Nodes
                i = 0
                current_group = -2
                vertex_group_count = 0
                for v in sorted_nodes:
                    # legacy style name augmentation using x sign
                    if v.x > 0:
                        v.node_name = v.node_name + 'l' + str(i)
                    elif v.x < 0:
                        v.node_name = v.node_name + 'r' + str(i)
                    else:
                        v.node_name = v.node_name + str(i)

                    # group change marker
                    vg = v.groups
                    gid = vg[0].group if len(vg) else -1
                    if gid != current_group:
                        current_group = gid
                        if gid != -1:
                            vertex_group_count += 1
                        f.write('\t\t')
                        # group name if available
                        group_name = ''
                        if gid != -1 and obj.vertex_groups and gid < len(obj.vertex_groups):
                            group_name = obj.vertex_groups[gid].name
                        f.write('{"group":"' + (group_name if vertex_group_count != 0 else part_name) + '"},\n')

                    f.write(f'\t\t["{v.node_name}",{v.x},{v.y},{v.z}],\n')
                    i += 1
                if current_group != -1 or vertex_group_count == 0:
                    f.write('\t\t{"group":""},\n')
                f.write('\t],\n')

                # Beams
                f.write('\t"beams":[\n\t\t["id1:","id2:"],\n')
                bm.edges.ensure_lookup_table()
                for e in bm.edges:
                    v1, v2 = e.verts
                    f.write(f'\t\t["{sorted_nodes[idx_map[v1.index]].node_name}","{sorted_nodes[idx_map[v2.index]].node_name}"],\n')
                # Auto diagonals for quad faces
                bm.faces.ensure_lookup_table()
                for face in bm.faces:
                    if len(face.verts) == 4:
                        v1,v2,v3,v4 = face.verts
                        f.write(f'\t\t["{sorted_nodes[idx_map[v1.index]].node_name}","{sorted_nodes[idx_map[v3.index]].node_name}"],\n')
                        f.write(f'\t\t["{sorted_nodes[idx_map[v2.index]].node_name}","{sorted_nodes[idx_map[v4.index]].node_name}"],\n')
                    elif len(face.verts) > 4:
                        self.report({'ERROR'}, 'Ngons not supported in legacy exporter')
                        break
                f.write('\t],\n')

                # Collision triangles
                f.write('\t"triangles":[\n\t\t["id1:","id2:","id3:"],\n')
                # Triangulate a copy for export
                tri_bm = bmesh.new()
                tri_bm.from_mesh(mesh)
                bmesh.ops.triangulate(tri_bm, faces=tri_bm.faces[:])
                tri_bm.faces.ensure_lookup_table()
                for face in tri_bm.faces:
                    if len(face.verts) == 3:
                        a,b,c = face.verts
                        f.write(f'\t\t["{sorted_nodes[idx_map[a.index]].node_name}","{sorted_nodes[idx_map[b.index]].node_name}","{sorted_nodes[idx_map[c.index]].node_name}"],\n')
                f.write('\t],\n')
                f.write('},\n}')

            bm.free()
            tri_bm.free()
            bpy.data.meshes.remove(mesh)
            self.report({'INFO'}, f'Legacy exported {filename}')

        return {'FINISHED'}

# Optional: simple menu helper (hooked in registration if desired)

def legacy_export_menu(self, context):
    self.layout.operator(JBEAM_EDITOR_OT_legacy_jbeam_export.bl_idname, text='Legacy JBeam Export (.jbeam)')
