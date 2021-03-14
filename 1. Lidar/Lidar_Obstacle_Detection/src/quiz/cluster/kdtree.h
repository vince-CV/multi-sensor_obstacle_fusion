/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}


	void insertRecursive(Node* &node, uint depth, std::vector<float> point, int id)
	{
		if (node == NULL)
			node = new Node(point, id);
		else
		{
			uint ui = depth % 2;
			if (point[ui] < node->point[ui]) 
				insertRecursive(node->left, depth+1, point, id);
			else
				insertRecursive(node->right, depth+1, point, id);
		}

	}


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertRecursive(root, 0, point, id);
	}

/*
	void insertRecursive(Node **node, int depth, std::vector<float> point, int id)
	{
		if (*node == nullptr)
		{
			*node = new Node(point, id);
		}
		else
		{
			int d = depth % 2;

			if( point[d] < (*node)->point[d] )
			{
				insertRecursive( &((*node)->left), depth++, point, id );
			}
			else
			{
				insertRecursive( &((*node)->right), depth++, point, id );
			}
		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		insertRecursive(&root, 0, point, id);

	}

*/
	
	bool checkInTargetBox(std::vector<float> src, std::vector<float> dst, float d)
	{
		if ((src[0]>=(dst[0]-d)) && (src[0]<=(dst[0]+d)) && (src[1]>=(dst[1]-d)) && (src[1]<=(dst[1]+d)) )
		{
			return true;
		}

		return false;
	}

	float getDistance(std::vector<float> src, std::vector<float> dst)
	{
		return sqrt((src[0]-dst[0])*(src[0]-dst[0]) + (src[1]-dst[1])*(src[1]-dst[1]));
	}
	
	
	void searchRecursive( Node* node, std::vector<float> target,int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node!=NULL)
		{
			if (checkInTargetBox(node->point, target, distanceTol))
			{
				if (getDistance(node->point, target) < distanceTol)
					ids.push_back(node->id);
			}

			//int di = depth % 2;
			if (target[depth % 2] - distanceTol  < node->point[depth % 2])
				searchRecursive(node->left, target, depth+1, distanceTol, ids);
			if (target[depth % 2] + distanceTol  > node->point[depth % 2])
				searchRecursive(node->right, target, depth+1, distanceTol, ids);
		}


	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchRecursive(root, target, 0, distanceTol, ids);
		return ids;
	}
	


};




