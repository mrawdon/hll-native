#pragma once
#ifndef BASE_64_CONVERTER
#define BASE_64_CONVERTER

// Base 64 encoding as defined on en.wikipedia.org/wiki/Base64

static char intToBase64(unsigned int val) {
	if (val <= 25) {
		return 'A' + val;
	} else if (val <= 51) {
		return 'a' + (val-26);
	} else if (val <= 61) {
		return '0' + (val-52);
	} else if (val == 62) {
		return '+';
	} else if (val == 63) {
		return '/';
	}

	return '-';
}

static unsigned int base64ToInt(char val) {
	if ('A' <= val && val <= 'Z') {
		return val - 'A';
	} else if ('a' <= val && val <= 'z') {
		return (val - 'a') + 26;
	} else if ('0' <= val && val <= '9') {
		return (val - '0') + 52;
	} else if (val == '+') {
		return 62;
	} else if (val == '/') {
		return 63;
	}

	return 64;
}

#endif
