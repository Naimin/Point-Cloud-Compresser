#pragma once
#include <string>

namespace CPC
{
    namespace Huffman
    {
        const int CHAR_LIMIT = 256;

        class Node 
        {
            public:
                Node() : leftC(nullptr), rightC(nullptr) {}
                Node(const Node &n) { data = n.data; frequency = n.frequency; leftC = n.leftC; rightC = n.rightC; }
                Node(unsigned char d, size_t f) : data(d), frequency(f), min_(d), leftC(nullptr), rightC(nullptr) {}
                Node(Node* rc, Node* lc);
                void fillCodebook(std::string* codebook, std::string& code);
                bool operator> (const Node& rhs);
             private:
                 unsigned char data;
                 size_t frequency;
                 unsigned char min_;
                 Node * leftC;
                 Node * rightC;
        };

        class Heap
        {
            public:
                Heap() { heapSize = 0; minHeap = new Node*[257]; } // max of 255 characters
                void push(Node *);
                int size() { return heapSize; }
                void pop();
                Node * top() { return minHeap[1]; }
            private:
                Node **minHeap;
                int heapSize;
        };

        class Huffman
        {
            public:
                Huffman();
                ~Huffman();

                bool decompress(const std::string& compressedFile, const std::string& decompressFile);
                bool compress(const std::string& inputFile, const std::string& compressedFile);

            protected:
                bool saveToFile(std::ifstream& inputStream, const std::string& compressedFile);
                Node * constructHeap();

            private:
                size_t frequencies[CHAR_LIMIT] = { 0 };
                std::string codebook[CHAR_LIMIT];
        };
    }
}