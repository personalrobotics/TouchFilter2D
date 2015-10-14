#ifndef BASICMAT_H_
#define BASICMAT_H_

#include <string>
#include <sstream>

namespace arm_slam
{
    // Extremely simple 2D matrix library. Templated sizes. No error checking. No optimization.
  template <size_t N, size_t M> class BasicMat
    {
        public:
            typedef BasicMat<N, M> Type;
            BasicMat()
            {
                   for(size_t i = 0; i < N * M; i++)
                   {
                       m[i] = 0.0f;
                   }
            }

            ~BasicMat()
            {

            }

            inline std::string ToString()
            {
                std::stringstream ss;

                for (size_t r = 0; r < N; r++)
                {
                    for (size_t c = 0; c < M; c++)
                    {
                        ss << (*this)(r, c) << " ";
                    }

                    ss << "\n";
                }

                return ss.str();
            }

            inline size_t GetNumRows()
            {
                return N;
            }

            inline size_t GetNumCols()
            {
                return M;
            }

            float& operator[](size_t idx)
            {
                return m[idx];
            }

            const float& operator[](size_t idx) const
            {
                return m[idx];
            }

            float& operator()(size_t r, size_t c = 0)
            {
                return (*this)[r * M + c];
            }

            const float& operator()(size_t r, size_t c  = 0) const
            {
                return (*this)[r * M + c];
            }

            void operator=(const Type& other)
            {
                for(size_t i = 0; i < N * M; i++)
                 {
                     m[i] = other[i];
                 }
            }

            void operator+=(const Type& other)
            {
                for(size_t i = 0; i < N * M; i++)
                {
                    m[i] += other[i];
                }
            }

            friend Type operator+(Type lhs, const Type& rhs)
            {
                Type toReturn = lhs;
                toReturn += rhs;
                return toReturn;
            }

            void operator-=(const Type& other)
            {
                for(size_t i = 0; i < N * M; i++)
                {
                    m[i] -= other[i];
                }
            }

            friend Type operator-(Type lhs, const Type& rhs)
            {
                Type toReturn = lhs;
                toReturn -= rhs;
                return toReturn;
            }


            Type operator*=(const float scalar)
            {
                Type toReturn = *this;
                for(size_t i = 0; i < N * M; i++)
                {
                    toReturn[i] *= scalar;
                }
                return toReturn;
            }

            friend Type operator*(Type lhs, const float& rhs)
            {
                return lhs *= rhs;
            }

            friend Type operator*(const float& lhs, Type rhs)
            {
                return rhs *= lhs;
            }

            inline BasicMat<M, N> Transpose() const
            {
                BasicMat<M, N> toReturn;

                for(size_t r = 0; r < N; r++)
                {
                    for(size_t c = 0; c < M; c++)
                    {
                        toReturn(c, r) = (*this)(r, c);
                    }
                }
                return toReturn;
            }

            template <size_t N2> BasicMat<N2, M> PreMult(const BasicMat<N2, N>& lhs)
            {
                    BasicMat<N2, M> toReturn;

                    for (size_t r = 0; r < N2; r++)
                    {
                        for(size_t c = 0; c < M; c++)
                        {
                            for(size_t k = 0; k < N; k++)
                            {
                                toReturn(r, c) += lhs(r, k) * (*this)(k, r);
                            }
                        }
                    }
                    return toReturn;
            }

            template <size_t M2> BasicMat<N, M2> PostMult(const BasicMat<M, M2>& rhs)
            {
                BasicMat<N, M2> toReturn;

                for (size_t r = 0; r < N; r++)
                {
                    for(size_t c = 0; c < M2; c++)
                    {
                        for(size_t k = 0; k < M; k++)
                        {
                            toReturn(r, c) += (*this)(r, k) * rhs(k, c);
                        }
                    }
                }
                return toReturn;
            }

            template <size_t M2> BasicMat<N, M2> operator*=(const BasicMat<M, M2>& rhs)
            {
                return PostMult(rhs);
            }

            template <size_t M2> friend BasicMat<N, M2> operator*(Type lhs, const BasicMat<M, M2>& rhs)
            {
                return lhs *= rhs;
            }

            float m[N * M];
    };

    typedef BasicMat<1, 1> Mat1x1;
    typedef BasicMat<2, 2> Mat2x2;
    typedef BasicMat<3, 3> Mat3x3;
    typedef BasicMat<4, 4> Mat4x4;
    typedef BasicMat<2, 1> Vec2;
    typedef BasicMat<3, 1> Vec3;
    typedef BasicMat<4, 1> Vec4;
    typedef BasicMat<5, 1> Vec5;
    typedef BasicMat<6, 1> Vec6;
    typedef BasicMat<7, 1> Vec7;
}

#endif // BASICMAT_H_ 
