#include "HllObject.h"
#include "hyperloglog.h"
#include "sds.h"
#include "HashingAlgorithms.h"
#include <math.h>
#include <vector>
#include <stdexcept>

HllObject::HllObject() :
	hllObject(NULL),
	totalCount(0),
	skipBytesFront(0),
	skipBytesBack(0)
{
}

#define FIXBOUND(X, Y, Z) ( ((X)<((Y)+(Z))) ? 0 : (X)-(Y)-(Z) )
HllObject::HllObject(uint32_t _skipBytesFront, uint32_t _skipBytesBack) :
	hllObject(),
	totalCount(0),
	skipBytesFront(_skipBytesFront),
	skipBytesBack(_skipBytesBack)
{
}

HllObject::HllObject(const HllObject& other) :
	totalCount(other.totalCount),
	skipBytesFront(other.skipBytesFront),
	skipBytesBack(other.skipBytesBack)
{
	// make a copy of the other object.
	if (other.isInitialized()) {
		destroyHLLObject(hllObject);
		hllObject = sdscatsds(sdsempty(), other.hllObject);
	}
}

HllObject& HllObject::operator=(const HllObject& other) {
	destroyHLLObject(hllObject);
	hllObject = sdscatsds(sdsempty(), other.hllObject);
	return *this;
}

bool HllObject::isInitialized() const {
	return hllObject != NULL;
}

HllObject::~HllObject()
{
	destroyHLLObject(hllObject);
}

uint64_t HllObject::getHash(const char data[], const int length) {
	const char* data_p = data + skipBytesFront;
	int len = length - skipBytesFront - skipBytesBack;
	int skippedBytes = skipBytesFront + skipBytesBack;

	return EnabledHash(data_p, len, 0x0);
}

bool HllObject::add(uint64_t hash) {
	init();

	totalCount++;

	// Add hash.
	mergeArray[0] = hllObject;
	int ret = hllAddHash(mergeArray, hash);
	hllObject = (char*)mergeArray[0];

	return (ret == 0);
}

bool HllObject::add(const char* data, const int length) {
	init();

	totalCount++;

	mergeArray[0] = hllObject;

	const char* data_p = data + skipBytesFront;
	int len = length - skipBytesFront - skipBytesBack;

	int skippedBytes = skipBytesFront + skipBytesBack;
	if (skippedBytes > length) {
		return true;
	}

	mergeArray[1] = const_cast<char*>(data_p);

	uint64_t hashOut;
	int ret = pfaddCommand((void**)mergeArray, 2, len, &hashOut);

	/*
	// TODO: Use EnabledHash and "hllAddHash" function.
	//uint64_t hashOut2 = EnabledHash(data_p, len, 0x0);
	uint64_t hashOut2 = getHash(data, length);
	if (hashOut != hashOut2)
		std::cout << "HASH ERROR\n";
	else
		std::cout << "HASH GOOD\n";
	*/

	hllObject = (char*)mergeArray[0];


	return (ret == 0);
}

bool HllObject::merge(const HllObject& other) {
	// If the other object is uninitialized, exit early.
	if (! other.isInitialized())
		return true;

	init();

	char* hllObjects[2];
	hllObjects[0] = hllObject;
	hllObjects[1] = other.hllObject;

	//int ret = mergeHllObjects(hllObjects, 2);
	int ret = pfmergeCommand((void**)hllObjects, 2);

	hllObject = hllObjects[0];
	totalCount += other.totalCount;

	return (ret == 0);
}

std::string HllObject::serialize() {
	if (hllObject == NULL) return "";

	double bufferLen = pow(2.0, (double)getP()) + 1;
	std::vector<char> vecBuffer(bufferLen);
	char* buffer = &vecBuffer[0];
	serializeHLLObject(hllObject, buffer, bufferLen);
	return std::string(buffer);
}

void HllObject::deserialize(const char* data) {
	init();
	double bufferLen = pow(2.0, (double)getP());
	mergeArray[0] = hllObject;
	deserializeHLLObject((char**)mergeArray, data, bufferLen);
	hllObject = (char*)mergeArray[0];
}

uint64_t HllObject::getCardinality() {
	if (hllObject == NULL) return 0;

	mergeArray[0] = hllObject;
	unsigned long long val = pfcountCommand(mergeArray, 1);
	hllObject = (char*)mergeArray[0];
	return val;
}

void HllObject::resetObject() {
	totalCount = 0;
}

void HllObject::init() {
	if (hllObject == NULL) {
		hllObject = (char*)createDenseHLLObject();
	}

	if (hllObject == NULL) {
		printf("Allocation error in Cardinality class.");
	}
}

