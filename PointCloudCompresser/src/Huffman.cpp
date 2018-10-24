#include "Huffman.h"
#include <iostream>
#include <fstream>
#include <limits>

using namespace CPC::Huffman;

void Node::fillCodebook(std::string * codebook, std::string &code) {
    if (!leftC && !rightC) {
        codebook[data] = code;
        return;
    }
    if (leftC) {
        code += '0';
        leftC->fillCodebook(codebook, code);
        code.erase(code.end() - 1);
    }
    if (rightC) {
        code += '1';
        rightC->fillCodebook(codebook, code);
        code.erase(code.end() - 1);
    }
}

Node::Node(Node * rc, Node * lc) : rightC(rc), leftC(lc) {
    frequency = rc->frequency + lc->frequency;
    min_ = (rc->min_ < lc->min_) ? rc->min_ : lc->min_;
}

void Heap::push(Node *newNode) {
    int currentHeapNode = ++heapSize;
    while (currentHeapNode != 1 && *minHeap[currentHeapNode / 2] > *newNode) {
        minHeap[currentHeapNode] = minHeap[currentHeapNode / 2];
        currentHeapNode = currentHeapNode / 2;
    }
    minHeap[currentHeapNode] = newNode;
}

void Heap::pop() {
    Node *lastNode = minHeap[heapSize];
    minHeap[heapSize--] = minHeap[1];
    int currentHeapNode = 1;
    int child = 2;

    while (child <= heapSize) {
        if (child < heapSize && *minHeap[child] > *minHeap[child + 1])
            child++;

        if (*minHeap[child] > *lastNode)
            break;

        minHeap[currentHeapNode] = minHeap[child];
        currentHeapNode = child;
        child *= 2;
    } // while not at end of heap

    minHeap[currentHeapNode] = lastNode;
}

bool Node::operator> (const Node& rhs) {
    if (frequency > rhs.frequency)
        return true;
    if (frequency < rhs.frequency)
        return false;
    if (frequency == rhs.frequency)
        if (min_ > rhs.min_)
            return true;
    return false;
}

CPC::Huffman::Huffman::Huffman()
{
}

CPC::Huffman::Huffman::~Huffman()
{
}

bool CPC::Huffman::Huffman::decompress(const std::string& compressedFile, const std::string& decompressFile)
{
    std::ifstream inFile(compressedFile, std::ifstream::binary);
    if (!inFile.is_open())
        return false;

    std::ofstream outFile(decompressFile, std::ofstream::binary);
    if (!outFile.is_open())
    {
        inFile.close();
        return false;
    }

    inFile >> std::noskipws;
    char magic[8];
    inFile.read(magic, 8);
    char nextByte;
    for (int i = 0; i < 256; i++) {
        inFile.read((char *)&frequencies[i], 4);
    }

    Node * root = constructHeap();
    std::string code;
    root->fillCodebook(codebook, code);

    while (inFile >> nextByte) 
    {
        for (int i = 0; i < 8; ++i) 
        {
            code += ((nextByte >> i) & 0x01) ? '1' : '0';

            for (int j = 0; j < 256; ++j) 
            {
                if (codebook[j] == code) 
                {
                    if (frequencies[j]) 
                    {
                        outFile << (unsigned char)j;
                        code.clear();
                        --frequencies[j];
                        break;
                    }
                    else
                        return false;
                }
            } // for
        }
    }

    inFile.close();
    outFile.close();

    return true;
}

bool CPC::Huffman::Huffman::compress(const std::string& inputFile, const std::string& compressedFile)
{
    std::ifstream inFile(inputFile, std::ifstream::binary);
    if (!inFile.is_open())
        return false;

    // Compute the frequencies of each byte in the file
    unsigned char nextChar;
    inFile >> std::noskipws;
    while (inFile >> nextChar)
        frequencies[nextChar]++;

    // Compute the codebook
    Node * root = constructHeap();
    std::string code;
    root->fillCodebook(codebook, code);

    // perform huffman encoding and save to file
    return saveToFile(inFile, compressedFile);
}

bool CPC::Huffman::Huffman::saveToFile(std::ifstream& inputStream, const std::string& compressedFile)
{
    std::ofstream outFile(compressedFile, std::ofstream::binary);
    if (!outFile.is_open())
        return false;

    outFile << "HUFFMA3" << '\0';

    unsigned int i;
    for (i = 0; i < 256; i++) {
        outFile << (char)(0x000000ff & frequencies[i]);
        outFile << (char)((0x0000ff00 & frequencies[i]) >> 8);
        outFile << (char)((0x00ff0000 & frequencies[i]) >> 16);
        outFile << (char)((0xff000000 & frequencies[i]) >> 24);
    }

    unsigned char nextChar;
    char nextByte = 0;
    int bitCounter = 0;

    inputStream.clear();
    inputStream.seekg(0);
    inputStream >> std::noskipws;
    while (inputStream >> nextChar) {
        for (i = 0; i < codebook[nextChar].size(); i++, bitCounter++) {
            if (bitCounter == 8) {
                outFile << nextByte;
                nextByte = 0;
                bitCounter = 0;
            }
            if (codebook[nextChar][i] == '1')
                nextByte = nextByte | (0x01 << bitCounter);
        }
    }
    if (bitCounter)
        outFile << nextByte;

    inputStream.close();
    outFile.close();

    return true;
}

Node * CPC::Huffman::Huffman::constructHeap()
{
    Heap minHeap;
    Node *nextNode;
    for (int i = 0; i < 256; i++) {
        if (frequencies[i]) {
            nextNode = new Node(i, frequencies[i]);
            minHeap.push(nextNode);
        }
    }

    Node * node1;
    Node * node2;
    Node * merged;
    while (minHeap.size() > 1) {
        node1 = minHeap.top();
        minHeap.pop();
        node2 = minHeap.top();
        minHeap.pop();
        merged = new Node(node1, node2);
        minHeap.push(merged);
    }

    return minHeap.top();
}
