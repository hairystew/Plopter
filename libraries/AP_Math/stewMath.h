#pragma once

#define PI 3.1415926535897932384626433832795
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>


#ifndef NDEBUG
#   define ASSERT(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message << std::endl; \
            std::terminate(); \
        } \
    } while (false)
#else
#   define ASSERT(condition, message) do { } while (false)
#endif





namespace sm {




    //-----------------------------GENERIC 3D VECTOR-----------------------------------------
    template <class T>
    struct vec3d_generic //Creating 3-D vector structure
    {
        T x = 0.0;
        T y = 0.0;
        T z = 0.0;

        vec3d_generic() : x(0), y(0), z(0) {}
        vec3d_generic(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
        vec3d_generic(const vec3d_generic& v) : x(v.x), y(v.y), z(v.z) {}


        //Adding utility functions

        T mag() {
            return sqrt(x * x + y * y + z * z);
        }

        vec3d_generic norm() {
            T r = 1 / mag();
            return vec3d_generic(x * r, y * r, z * r);
        }


        //Using operator overloading to allow the vec3d structor to behave like a primative

        vec3d_generic operator + (const vec3d_generic& rightSide)
        {
            return vec3d_generic(this->x + rightSide.x, this->y + rightSide.y, this->z + rightSide.z);
        }

        vec3d_generic operator - (const vec3d_generic& rightSide)
        {
            return vec3d_generic(this->x - rightSide.x, this->y - rightSide.y, this->z - rightSide.z);
        }

        vec3d_generic& operator += (const vec3d_generic& rightSide)
        {
            this->x += rightSide.x; this->y += rightSide.y; this->z += rightSide.z;
            return *this;
        }

        vec3d_generic& operator -= (const vec3d_generic& rightSide)
        {
            this->x -= rightSide.x; this->y -= rightSide.y; this->z -= rightSide.z;
            return *this;
        }

        vec3d_generic& operator *= (const T rightSide) //Multiplying by a scalar
        {
            this->x -= rightSide; this->y -= rightSide; this->z -= rightSide;
            return *this;
        }

        vec3d_generic& operator /= (const T rightSide) //Dividing by a scalar
        {
            this->x -= rightSide; this->y -= rightSide; this->z -= rightSide;
            return *this;
        }

        vec3d_generic operator * (const T rightSide) //Multiplying by a scalar
        {
            return vec3d_generic(this->x * rightSide, this->y * rightSide, this->z * rightSide);
        }



        vec3d_generic operator / (const T rightSide) //Dividing by a scalar
        {
            return vec3d_generic(this->x / rightSide, this->y / rightSide, this->z / rightSide);
        }


    };

    typedef vec3d_generic<double> vec3d;

    // overloading "<<" operator to allow for ease of printing to console, i.e "std::cout << genericVector"
    // I believe it is good practice to overload input operators to the iostream library as a nonmember function, however I could be wrong

    template <class T>
    std::ostream& operator<<(std::ostream& os, const vec3d_generic<T>& vec) {
        os << "{" << vec.x << ", " << vec.y << ", " << vec.z << "}";
        return os;
    }

    // Creating funcitons for cross product and dot product
    template <class T>
    vec3d_generic<T> cross(const vec3d_generic<T> vec1, const vec3d_generic<T> vec2) {
        return{ (vec1.y * vec2.z - vec2.y * vec1.z), -(vec1.x * vec2.z - vec2.x * vec1.z), (vec1.x * vec2.y - vec2.x * vec1.y) };
    }
    template <class T>
    double dot(const vec3d_generic<T> vec1, const vec3d_generic<T> vec2) {
        return(vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z);
    }

    template <class T>
    vec3d_generic<T> rot(const vec3d_generic<T> vec, const double angle, const char rotAxis) // angle in degrees, use single quotes when defining rotAxis... i.e. 'x' 
    {
        switch (rotAxis) {
        case 'x':
            return { vec.x, vec.y * cos(angle * PI / 180) - vec.z * sin(angle * PI / 180), vec.y * sin(angle * PI / 180) + vec.z * cos(angle * PI / 180) };
        case 'y':
            return { vec.x * cos(angle * PI / 180) + vec.z * sin(angle * PI / 180) ,vec.y ,-vec.x * sin(angle * PI / 180) + vec.z * cos(angle * PI / 180) };
        case 'z':
            return { vec.x * cos(angle * PI / 180) - vec.y * sin(angle * PI / 180), vec.x * sin(angle * PI / 180) + vec.y * cos(angle * PI / 180), vec.z };
        }
    }



    // MATRIX OPERATIONS

    template <class T>
    class Matrix
    {

    private:
        T **data; //Values in the Matrix

        //for eigenval stuff

    public:
        //reduce matrix to upper Hessenberg form
        //Come back in clean this up, not very memory efficient. Rework how matrix data is
        //stored to allow for faster copying of info.
        Matrix<T> upperHessenberg() {
            ASSERT(n == m, "MATRIX NOT SQUARE!!!");
            Matrix<T> carry(*this);
            Matrix<T> eigvecs(n);
            for (unsigned i = 0; i < n - 2; i++) {
                Matrix<T> _x(n - i - 1, 1, 0);
                Matrix<T> _w(_x);
                Matrix<T> _v(_x);
                for (unsigned j = i + 1; j < n; j++) {
                    _x(j - i - 1, 0) = carry.data[j][i];
                }
                
                _w(0, 0) = (_x(0, 0) > 0) ? -_x.colMag(0) : _x.colMag(0);
                _v = _w - _x;
                Matrix<T> _p(n - i - 1);
                _p = (_v * _v.transpose()) * (1 / (_v.transpose() * _v)(0, 0));
                //create reflector
                Matrix<T> _hTemp = Matrix<T>(n - i - 1) - (_p * 2);
                Matrix<T> _h(n);
                for (unsigned k = i + 1; k < n; k++) {
                    for (unsigned l = i + 1; l < n; l++) {
                        _h(k, l) = _hTemp(k - i - 1, l - i - 1);
                    }
                }
                carry = _h * (carry * _h);
            }
            return carry;
        }




        //reduce matrix to upper Schur form using QR algorithm
        std::vector<Matrix<T>> qrDecomp() {
            ASSERT(n == m, "MATRIX NOT SQUARE!!!");
            Matrix<T> Q(n);
            Matrix<T> R(n);
            for (unsigned i = 0; i < n; i++) {
                Matrix<T> col(n, 1, 0);
                Matrix<T> modCol(n, 1, 0);
                for (unsigned k = 0; k < n; k++) {
                    col(k, 0) = this->data[k][i];
                }
                for (unsigned j = 0; j <= i; j++) {
                    if (j == i) {
                        for (unsigned k = 0; k < n; k++) {
                            modCol(k, 0) += col(k, 0);
                        }
                        R(j, i) = modCol.colMag(0);
                    }
                    else {
                        //come back and make a better interop between vectors and matricies
                        T dotVal = 0;
                        dotVal = 0;
                        //dot prod
                        for (unsigned k = 0; k < n; k++) {
                            dotVal += (col(k, 0) * Q(k, j));
                        }
                        for (unsigned l = 0; l < n; l++) {
                            modCol(l, 0) -= Q(l, j) * dotVal;
                        }
                        R(j, i) = dotVal;
                    }
                }

                for (unsigned k = 0; k < n; k++) {
                    Q(k, i) = modCol(k, 0) * (1. / modCol.colMag(0));
                }
            }
            std::vector<Matrix<T>> ret;
            ret.push_back(Q);
            ret.push_back(R);
            return ret;
        }
        //return eigenvalues

        //uses givens rotations to make an upper triangular matrix from an upper hessenberg matrix
        std::vector<Matrix<T>> qrDecompHessenberg() {
            ASSERT(n == m, "MATRIX NOT SQUARE!!!");
            Matrix<T> r(*this);
            Matrix<T> q(this->n);
            for (unsigned i = 0; i < n - 1; i++) {
                Matrix<T> rot(n);
                T rad = sqrt(pow(r(i, i), 2) + pow(r(i + 1, i), 2));
                T c = r(i, i) / rad;
                T s = -r(i + 1, i) / rad;
                rot(i, i) = c;
                rot(i + 1, i) = s;
                rot(i, i + 1) = -s;
                rot(i + 1, i + 1) = c;
                r = rot * r;
                q = q * rot.transpose();
            }
            std::vector<Matrix<T>> ret;
            ret.push_back(q);
            ret.push_back(r);
            return ret;
        }


        unsigned n; //Rows
        unsigned m; //Cols


        T& operator()(const unsigned& _n, const unsigned& _m) {
            return this->data[_n][_m];
        }

        const T& operator()(const unsigned& _n, const unsigned& _m) const{
            return this->data[_n][_m];
        }

        //Initialize empty matrix
        Matrix<T>() : Matrix<T>(1) {
        }


        //Initialized a N x M matrix to _init (N rows and M cols)
        Matrix<T>(int _n, int _m, const T& _init) : n(_n), m(_m) {
            data = new T* [n];
            for (unsigned i = 0; i < n; ++i) {
                data[i] = new T[m];
                for (unsigned j = 0; j < m; j++) {
                    data[i][j] = _init;
                }
            }

        }

        //Initialized a square matrix to identity
        Matrix<T>(unsigned _n) : n(_n), m(_n) {
            data = new T * [n];
            for (unsigned i = 0; i < n; ++i) {
                data[i] = new T[m];
                for (unsigned j = 0; j < m; j++) {
                    if (i == j) { data[i][j] = 1; }
                    else { data[i][j] = 0; }
                }
            }

        }

        //copy constructor
        Matrix<T>(const Matrix<T>& rhs) {
            n = rhs.n;
            m = rhs.m;
            data = new T * [n];
            for (unsigned i = 0; i < n; ++i) {
                data[i] = new T[m];
                for (unsigned j = 0; j < m; j++) {
                    data[i][j] = rhs.data[i][j];
                }
            }
        }

        ~Matrix<T>() {
            for (unsigned i = 0; i < n; ++i) {
                delete[] data[i];
            }
            delete[] data;
        }

        Matrix<T> operator=(const Matrix<T>& rhs) {
            if (&rhs == this)
                return *this;

            for (unsigned i = 0; i < n; ++i) {
                delete[] data[i];
            }
            delete[] data;

            n = rhs.n;
            m = rhs.m;

            data = new T * [n];
            for (unsigned i = 0; i < n; ++i) {
                data[i] = new T[m];
                for (unsigned j = 0; j < m; j++) {
                    data[i][j] = rhs(i, j);
                }
            }
            return *this;
        }


        const void print() const{
            for (unsigned i = 0; i < n; i++) {
                for (unsigned j = 0; j < m; j++) {
                    std::cout << data[i][j] << ",\t";
                }
                std::cout << std::endl;
            }
        }

        void print() {
            for (unsigned i = 0; i < n; i++) {
                for (unsigned j = 0; j < m; j++) {
                    std::cout << data[i][j] << ",\t";
                }
                std::cout << std::endl;
            }
        }


        Matrix<T> transpose() {
            //ASSERT(n == m, "MATRIX NOT SQUARE");
            Matrix<T> res(m, n, 0);
            for (unsigned i = 0; i < n; i++) {
                for (unsigned j = 0; j < m; j++) {
                    res(j, i) = data[i][j];
                }
            }
            return res;
        }

        //matrix addition
        Matrix<T> operator+(const Matrix<T>& rhs) {
            ASSERT(rhs.n == n || rhs.m == m, "MATRIX SIZE MISMATCH");
            Matrix<T> res(n ,m , 0);
            for (unsigned i = 0; i < n; i++) {
                for (unsigned j = 0; j < m; j++) {
                    res(i, j) = rhs(i, j) + this->data[i][j];
                }
            }
            return res;
        }

        //scalar addition
        Matrix<T> operator+(const T& rhs) {
            ASSERT(rhs.n == n || rhs.m == m, "MATRIX SIZE MISMATCH");
            Matrix<T> res(n, m, 0);
            for (unsigned i = 0; i < n; i++) {
                for (unsigned j = 0; j < m; j++) {
                    res(i, j) = rhs + this->data[i][j];
                }
            }
            return res;
        }

        //matrix subtraction
        Matrix<T> operator-(const Matrix<T>& rhs) {
            ASSERT(rhs.n == n || rhs.m == m, "MATRIX SIZE MISMATCH");
            Matrix<T> res(n, m, 0);
            for (unsigned i = 0; i < n; i++) {
                for (unsigned j = 0; j < m; j++) {
                    res(i, j) = this->data[i][j] - rhs(i, j);
                }
            }
            return res;
        }

        //scalar subtraction
        Matrix<T> operator-(const T& rhs) {
            ASSERT(rhs.n == n || rhs.m == m, "MATRIX SIZE MISMATCH");
            Matrix<T> res(n, m, 0);
            for (unsigned i = 0; i < n; i++) {
                for (unsigned j = 0; j < m; j++) {
                    res(i, j) = this->data[i][j] - rhs;
                }
            }
            return res;
        }

        //matrix addition
        void operator+=(const Matrix<T>& rhs) {
            ASSERT(rhs.n == n || rhs.m == m, "MATRIX SIZE MISMATCH");
            for (unsigned i = 0; i < n; i++) {
                for (unsigned j = 0; j < m; j++) {
                    this->data[i][j] += rhs(i, j);
                }
            }
        }

        //scalar addition
        void operator+=(const T& rhs) {
            ASSERT(rhs.n == n || rhs.m == m, "MATRIX SIZE MISMATCH");
            for (unsigned i = 0; i < n; i++) {
                for (unsigned j = 0; j < m; j++) {
                    this->data[i][j] += rhs;
                }
            }
        }

        //matrix subtraction
        void operator-=(const Matrix<T>& rhs) {
            ASSERT(rhs.n == n || rhs.m == m, "MATRIX SIZE MISMATCH");
            for (unsigned i = 0; i < n; i++) {
                for (unsigned j = 0; j < m; j++) {
                    this->data[i][j] -= rhs(i, j);
                }
            }
        }

        //scalar subtraction
        void operator-=(const T& rhs) {
            ASSERT(rhs.n == n || rhs.m == m, "MATRIX SIZE MISMATCH");
            for (unsigned i = 0; i < n; i++) {
                for (unsigned j = 0; j < m; j++) {
                    this->data[i][j] -= rhs;
                }
            }
        }

        //scalar multiplication
        Matrix<T> operator*(const T& rhs) {
            Matrix<T> res(this->n, this->m, 0);

            for (unsigned i = 0; i < this->m; i++) {
                for (unsigned j = 0; j < this->n; j++) {
                    res(i, j) += this->data[i][j] * rhs;
                }
            }
            return res;
        }

        //scalar multiplication
        void operator*=(const T& rhs) {
            Matrix<T> res = (*this) * rhs;
            (*this) = res;
        }

        //matrix multiplication
        Matrix<T> operator*(const Matrix<T>& rhs) {
            ASSERT(this->m == rhs.n, "MATRIX SIZE MISMATCH");
            Matrix<T> res(this->n, rhs.m, 0);
            for (unsigned i = 0; i < rhs.m; i++) {//out col
                for (unsigned j = 0; j < this->n; j++) { // out row
                    for (unsigned k = 0; k < rhs.n; k++) { // sum
                        res(j, i) += this->data[j][k] * rhs(k, i);
                    }
                }
            }
            return res;
        }

        //matrix multiplication
        void operator*=(const Matrix<T>& rhs) {
            Matrix<T> res = (*this) * rhs;
            (*this) = res;
        }

        T colMag(unsigned colIdx) {
            T ret = 0;
            for (unsigned i = 0; i < n; i++) {
                T val = this->data[i][colIdx];
                ret = ret + pow(val, 2);
            }
            return sqrt(ret);
        }


        //Eigen Value Calcualtion
        static std::vector<Matrix<T>> eigen(Matrix<T> mat, unsigned n = 50) {
            //ToDo:: use upper hessenberg and rotations to calculate both more efficiently
            Matrix<T> temp = mat.upperHessenberg();
            std::vector<sm::Matrix<double>> qr = temp.qrDecompHessenberg();
            Matrix<T> eigenvecs(mat.n);
            temp = qr[1] * qr[0];
            for (int i = 0; i < n; i++) {
                qr = temp.qrDecompHessenberg();
                temp = qr[1] * qr[0];
            }
            for (int i = 0; i < mat.n; i++) {
                sm::Matrix<T> tempHess = ((mat - (Matrix<T>(mat.n) * temp(i, i))).transpose()).upperHessenberg();
                std::vector<sm::Matrix<double>> qrEig = tempHess.qrDecompHessenberg();
                for (int j = 0; j < mat.n; j++) {
                    eigenvecs(j, i) = qrEig[0](j, mat.n - 1);
                }
            }
            std::vector<Matrix<T>> ret;
            ret.push_back(temp);
            ret.push_back(eigenvecs);
            return ret;
        }




        /*
        * Matrix Inversion Using Cholesky Decomposition Based on https://arxiv.org/ftp/arxiv/papers/1111/1111.4144.pdf
        */

        static Matrix<T> invOfCD(const Matrix<T>& A) {
            auto R = cd(A);
            //back-substitution

            for (int i = (A.n - 1); i >= 0; i--) {
                R(i, i) = (1 / R(i, i));
                for (int j = i - 1; j >= 0; j--) {
                    T sum = 0;
                    for (int k = i; k > j; k--) {
                        sum -= R(j, k) * R(k, i);
                    }
                    R(j, i) = (sum / R(j, j));
                }
            }

            return R;
        }

        //Cholesky Decomposition to get upper triangular matrix R
        static Matrix<T> cd(const Matrix<T>& A) {
            ASSERT(A.n == A.m, "MATRIX NOT SQUARE");
            Matrix<T> R(A.n, A.n, 0);
            for (unsigned i = 0; i < R.n; i++) {
                for (unsigned j = i; j < R.n; j++) {
                    T sum = 0;

                    //get diagonal elemets
                    if (i == j) {
                        for (unsigned k = 0; k < i; k++) {
                            sum += (R(k, i) * R(k, i));
                        }
                        R(i, i) = sqrt(A(i, i) - sum);
                    }

                    //get upper triangular values
                    else {
                        for (unsigned k = 0; k < i; k++) {
                            sum += (R(k, i) * R(k, j));
                        }
                        R(i, j) = (A(i, j) - sum) / R(i, i);
                    }
                }
            }
            return R;
        }

        static Matrix<T> cdInv(const Matrix<T>& A) {
            Matrix<T> R(A.n);
            auto Acdi = invOfCD(A);
            auto Acdit = Acdi.transpose();
            R = Acdi * Acdit;
            return R;
        }



        static void test() {
            Matrix<T> joe(3);
            Matrix<T> joel(3);
            joel(0, 0) = 4;
            joe(0, 0) = joel(0, 0);
            joe(0, 0) = 2;
            joel.print();

        }

        /*
        PSEUDO TODO
        adjoint
        cofactor


        */



    };


    template <class T>
    class LQR {
    public:
        Matrix<T> A;
        Matrix<T> B;
        Matrix<T> C;
        Matrix<T> D;

        //Cost Matricies For Optimal Gain Matrix K
        Matrix<T> Q;
        Matrix<T> R;

        //Optimal Gain Matrix K
        Matrix<T> K;

        //Calculate and Populate Optimal Gain Matrix K
        void calcK(Matrix<T> A, Matrix<T> B, Matrix<T> Q, Matrix<T> R) {
            /*
            Pseudo Code
            Create Hamiltonian Matrix
            Find eigenvalues and eigen vectors of hamiltonian matrix
            calculate K
            */

            //create hamiltonian
            Matrix<T> topRight = -B * R.cdInv() * B.transpose();

            Matrix<T> H(2 * A.n);
            for (int i = 0; i < 2 * A.n - 1; i++) {
                for (int j = 0; j < 2 * A.n - 1; j++) {
                    if (i < A.n) {
                        if (j < A.n) {
                            
                        }
                        else {

                        }
                    }
                    else {
                        if (j < A.n) {

                        }
                        else {

                        }
                    }
                }
            }
        }


    private:



    protected:



    };


}
