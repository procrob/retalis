/****
    This is part of the Retalis Language for Information Processing and Management in Robotics
    Copyright (C) 2014 __Pouyan Ziafati__ pziafati@gmail.com 
    Copyright (C) 2014 __Maciej Zurad__ maciej.zurad@gmail.com 

    Retalis is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Retalis is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.                   

    You should have received a copy of the GNU General Public License
    along with Retalis.  If not, see <http://www.gnu.org/licenses/>.	
****/





#include <iostream>
#include "SWI-cpp.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>


using namespace std;

template<class T> T read_term(term_t term);
template<> glm::mat4 read_term(term_t term);

template<class T> PlTerm construct_term(T& object);
template<> PlTerm construct_term(glm::mat4& matrix);

void print(glm::mat4 mat);
void print(glm::vec3 vec);
void print(glm::quat quat);

PREDICATE(print_matrix, 1)
{
	glm::mat4 load_matrix= read_term<glm::mat4>(A1);
	print(load_matrix);
	return TRUE;
}

PREDICATE(mul_matrix, 3)
{
	glm::mat4 left_matrix  = read_term<glm::mat4>(A1);
	glm::mat4 right_matrix = read_term<glm::mat4>(A2);
	
	glm::mat4 result = left_matrix * right_matrix;
	
	#ifdef DEBUG
		cout << "Begin multiplication" << endl;
		cout << "--------------------------" << endl;
		print(left_matrix);
		cout << "--------------------------" << endl;
		print(right_matrix);
		cout << "--------------------------" << endl;
		print(result);
		cout << "--------------------------" << endl;
		cout << "End multiplication" << endl;
	#endif

	return A3 = construct_term<glm::mat4>(result);
}


struct qqq{
		glm::vec3 a1;
		glm::quat a2;
	};

PREDICATE(mul_quaternion,5)
{
	glm::vec3 translation1 = read_term<glm::vec3>(A1);
	glm::quat rotation1    = read_term<glm::quat>(A2);
	glm::vec3 translation2 = read_term<glm::vec3>(A3);
	glm::quat rotation2    = read_term<glm::quat>(A4);
	
	glm::vec3 resTranslation = translation1 + translation2; 
	glm::quat resRotation    = rotation1 * rotation2; 
	
	PlTerm outerTerm;
	PlTail outerTail(outerTerm);
	
	PlTerm innerTerm;
	PlTail innerTail(innerTerm);
		
	innerTail.append(resTranslation.x);
	innerTail.append(resTranslation.y);
	innerTail.append(resTranslation.z);
	innerTail.close();		
	
	PlTerm innerTerm1;
	PlTail innerTail1(innerTerm1);
	innerTail1.append(resRotation.x);
	innerTail1.append(resRotation.y);	
	innerTail1.append(resRotation.z);
	innerTail1.append(resRotation.w);
	innerTail1.close();


	outerTail.append(innerTerm);
	outerTail.append(innerTerm1);
	outerTail.close();	
	
	return A5 = outerTerm; 
	 	
	 
}


/*
PREDICATE(convert_to_quat,2)
{	glm::mat4 matrixx= read_term<glm::mat4>(A1);
	glm::quat rotation    = glm::gtc::quaternion::quat_cast(matrixx);
	return A2 = construct_term((float)matrixx[0][3],(float)matrixx[1][3],(float)matrixx[1][3], (float)rotation[0],(float)rotation[1],(float)rotation[2],(float)rotation[3]);

}
*/


PREDICATE(interpolate_quaternion,6)
{
	glm::vec3 translation1 = read_term<glm::vec3>(A1);
	glm::quat rotation1    = read_term<glm::quat>(A2);
	glm::vec3 translation2 = read_term<glm::vec3>(A3);
	glm::quat rotation2    = read_term<glm::quat>(A4);
	PlTerm element;
	PlTail tail(A5);
	float t;
	if(tail.next(element)) 	t = (double)(element);
	glm::vec3 resTranslation = translation1 * t + translation2 * (1.f-t); 
	glm::quat resRotation =    glm::fastMix(rotation1, rotation2,t); 


	PlTerm outerTerm;
	PlTail outerTail(outerTerm);
	
	PlTerm innerTerm;
	PlTail innerTail(innerTerm);
		
	innerTail.append(resTranslation.x);
	innerTail.append(resTranslation.y);
	innerTail.append(resTranslation.z);
	innerTail.close();		
	
	PlTerm innerTerm1;
	PlTail innerTail1(innerTerm1);
	innerTail1.append(resRotation.x);
	innerTail1.append(resRotation.y);	
	innerTail1.append(resRotation.z);
	innerTail1.append(resRotation.w);
	innerTail1.close();


	outerTail.append(innerTerm);
	outerTail.append(innerTerm1);
	outerTail.close();	
	
	return A6 = outerTerm; 
	 	
	 
}

PREDICATE(quaternion_interpolate,6)
{
	glm::vec3 translation1 = read_term<glm::vec3>(A1);
	glm::quat rotation1    = read_term<glm::quat>(A2);
	glm::vec3 translation2 = read_term<glm::vec3>(A3);
	glm::quat rotation2    = read_term<glm::quat>(A4);
	PlTerm element;
	PlTail tail(A5);
	float t;
	if(tail.next(element)) 	t = (double)(element);
	glm::vec3 resTranslation = translation1 * t + translation2 * (1.f-t); 
	glm::quat resRotation =    glm::mix(rotation1, rotation2,t); 


	glm::mat4 rotation_matrix    = glm::toMat4(resRotation);
	glm::mat4 translation_matrix = glm::translate(glm::mat4(1.0f), resTranslation);

	glm::mat4 converted = translation_matrix * rotation_matrix;
	return A6 = construct_term<glm::mat4>(converted); 
	 	
	 
}




PREDICATE(convert_to_mat, 3)
{
	glm::vec3 translation = read_term<glm::vec3>(A1);
	glm::quat rotation    = read_term<glm::quat>(A2);

	glm::mat4 rotation_matrix    = glm::toMat4(rotation);
	glm::mat4 translation_matrix = glm::translate(glm::mat4(1.0f), translation);

	glm::mat4 converted = translation_matrix * rotation_matrix;

	#ifdef DEBUG
		cout << "Begin conversion to matrix" << endl;
		cout << "--------------------------" << endl;
		print(translation);
		cout << "--------------------------" << endl;
		print(rotation);
		cout << "--------------------------" << endl;
		print(converted);
		cout << "--------------------------" << endl;
		cout << "End conversion to matrix" << endl;
	#endif

	return A3 = construct_term<glm::mat4>(converted); 
}

PREDICATE(transform_point, 3)
{
	glm::vec3 point  = read_term<glm::vec3>(A1);
	glm::mat4 matrix = read_term<glm::mat4>(A2);

	glm::vec4 _translated = matrix * glm::vec4(point.x, point.y, point.z, 1);
	
	glm::vec3 translated = glm::vec3(_translated.x,
	 								 _translated.y,
 								     _translated.z);

	#ifdef DEBUG
		cout << "Begin point tf" << endl;
		cout << "--------------------------" << endl;
		print(point);
		cout << "--------------------------" << endl;
		print(matrix);
		cout << "--------------------------" << endl;
		print(translated);
		cout << "--------------------------" << endl;
		cout << "End point tf" << endl;
	#endif

	return A3 = construct_term<glm::vec3>(translated);
}

template<class T>
T read_term(term_t term)
{
	T t;
	int index = 0;
	PlTerm element;
	PlTail tail(term);
	while(tail.next(element)) {
		t[index] = (double)(element);
		index++;
	}
	return t;
}

template<>
glm::mat4 read_term(term_t term)
{
	glm::mat4 mat(1.0f);

	PlTerm innerList;
	PlTail outerTail(term);
	int i = 0;
	while(outerTail.next(innerList)) {
		int j = 0;
		PlTerm element;
		PlTail innerTail(innerList);
		while(innerTail.next(element)) {
			mat[j][i] = (double)(element);
			j++;
		}
		i++;
	}
	return mat;
}

template<class T>
PlTerm construct_term(T& object)
{
	PlTerm term;
	PlTail tail(term);
	for (int i = 0; i < object.length(); ++i)
	{
		tail.append(object[i]);
	}
	tail.close();
	return term;
}


/*
template<>
PlTerm construct_term(qqq& xxx)
{
	PlTerm outerTerm;
	PlTail outerTail(outerTerm);
	
	PlTerm innerTerm;
	PlTail innerTail(innerTerm);
		for (int j = 0; j < 3; ++j)
		{
			innerTail.append(xxx.a1.[j]);
		}
		innerTail.close();
		outerTail.append(innerTerm);
	
		PlTerm innerTerm1;
		PlTail innerTail1(innerTerm1);

		for (int j = 0; j < 4; ++j)
		{
			innerTail.append(xxx.a2.[j]);
		}
		innerTail.close();
		outerTail.append(innerTerm1);

	outerTail.close();
	return outerTerm;
}*/

template<>
PlTerm construct_term(glm::mat4& matrix)
{
	PlTerm outerTerm;
	PlTail outerTail(outerTerm);
	for (int i = 0; i < 4; ++i)
	{
		PlTerm innerTerm;
		PlTail innerTail(innerTerm);
		for (int j = 0; j < 4; ++j)
		{
			innerTail.append(matrix[j][i]);
		}
		innerTail.close();
		outerTail.append(innerTerm);
	}
	outerTail.close();
	return outerTerm;
}

void print(glm::mat4 mat)
{
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			cout << mat[j][i] << "\t";
		}
		cout << endl;
	}
}

void print(glm::vec3 vec)
{
	for (int i = 0; i < 3; ++i)
	{
		cout << vec[i];
	}
	cout << endl;
}

void print(glm::quat quat)
{
	for (int i = 0; i < 4; ++i)
	{
		cout << quat[i];
	}
	cout << endl;
}
