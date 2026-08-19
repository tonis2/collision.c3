<*
 Shared fixtures for the suite: the two meshes almost every soft-body check
 starts from, the ground plane the rigid-body ones fall onto, and the box
 emitter the voxel tests build their scenes with.

 Not a `@test` module, so nothing here is collected as a test — which is the
 whole reason it is a file of its own. Under `module collision_tests @test;`
 every fn in the file was a test, and a helper could only be written as a macro
 to stay out of the run. Each of these is still a macro because that is what it
 was; a fn would work now.

 **Everything here allocates, on `mem`, deliberately.** Each fixture calls
 `Mesh.init(mem)` before pushing anything. Left to itself a zero-valued `List`
 binds to `tmem` on its first push, and the runner's leak check only sees what
 goes through `mem` — so a suite whose fixtures were unseated would report
 leak-clean whatever the library did with them. Every mesh below is therefore
 the caller's to free: `defer mesh.free()` on the line after, or `c3c test`
 reports it against whichever test took it.
*>
module collision_testutil;

import collision;
import std::collections::list;

alias Vec3 = float[<3>];
alias BoxPoints = List {Vec3};
alias Triangles = List {Triangle};

// Helper: create a simple tetrahedron mesh for soft body testing
macro Mesh create_test_tetrahedron()
{
	Mesh mesh;
	mesh.init(mem, 4, 4);
	// Tetrahedron vertices
	mesh.vertices.push_all(
		{
			{ 0.0f, 0.0f, 1.0f }, // Top
			{ -1.0f, -1.0f, 0.0f }, // Base left
			{ 1.0f, -1.0f, 0.0f }, // Base right
			{ 0.0f, 1.0f, 0.0f }, // Base back
		}
	);

	// 4 triangular faces
	mesh.triangles.push_all({ { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 1 }, { 1, 3, 2 }, });
	return mesh;
}

// Helper: create a simple cube mesh for soft body testing
macro Mesh create_test_cube()
{
	Mesh mesh;
	mesh.init(mem, 8, 12);
	mesh.vertices.push_all(
		{
			{ -0.5f, -0.5f, -0.5f },
			{ 0.5f, -0.5f, -0.5f },
			{ 0.5f, 0.5f, -0.5f },
			{ -0.5f, 0.5f, -0.5f },
			{ -0.5f, -0.5f, 0.5f },
			{ 0.5f, -0.5f, 0.5f },
			{ 0.5f, 0.5f, 0.5f },
			{ -0.5f, 0.5f, 0.5f },
		}
	);
	mesh.triangles.push_all(
		{ // Front
			{ 0, 1, 2 },
			{ 0, 2, 3 }, // Back
			{ 5, 4, 7 },
			{ 5, 7, 6 }, // Top
			{ 3, 2, 6 },
			{ 3, 6, 7 }, // Bottom
			{ 4, 5, 1 },
			{ 4, 1, 0 }, // Right
			{ 1, 5, 6 },
			{ 1, 6, 2 }, // Left
			{ 4, 0, 3 },
			{ 4, 3, 7 },
		}
	);
	return mesh;
}

// Helper: flat two-triangle ground mesh at z=0 spanning ±10, with BVH
macro Mesh create_test_ground_mesh()
{
	Mesh mesh;
	mesh.init(mem, 4, 2);
	mesh.vertices.push_all(
		{
			{ -10.0f, -10.0f, 0.0f },
			{ 10.0f, -10.0f, 0.0f },
			{ -10.0f, 10.0f, 0.0f },
			{ 10.0f, 10.0f, 0.0f },
		}
	);
	// Consistent winding — face normals point up (+z)
	mesh.triangles.push_all({ { 0, 1, 2 }, { 1, 3, 2 } });
	mesh.aabb = mesh.get_aabb();
	mesh.build_bvh();
	return mesh;
}

// A closed axis-aligned box, twelve triangles, appended to the lists given.
macro void push_box(BoxPoints* vertices, Triangles* triangles, Vec3 low, Vec3 high)
{
	uint first = (uint)vertices.len();
	vertices.push_all({
		{ low.x, low.y, low.z },   { high.x, low.y, low.z },
		{ high.x, high.y, low.z }, { low.x, high.y, low.z },
		{ low.x, low.y, high.z },  { high.x, low.y, high.z },
		{ high.x, high.y, high.z }, { low.x, high.y, high.z },
	});

	uint[36] faces = {
		0, 1, 2,  0, 2, 3,   // -z
		4, 6, 5,  4, 7, 6,   // +z
		0, 5, 1,  0, 4, 5,   // -y
		3, 2, 6,  3, 6, 7,   // +y
		0, 3, 7,  0, 7, 4,   // -x
		1, 5, 6,  1, 6, 2,   // +x
	};
	for (usz i = 0; i < 36; i += 3)
	{
		triangles.push({ first + faces[i], first + faces[i + 1], first + faces[i + 2] });
	}
}

fn int slot_worker(void* arg)
{
	*(int*)arg += 1;
	return 0;
}
