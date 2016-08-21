#include <gtest/gtest.h>
#include <labust/tools/bitstorage.h>
#include <cstdint>

using labust::tools::BitStorage;

class BitStorageTest : public ::testing::Test
{
protected:
  template <class Type>
  struct Element
  {
    typedef Type BaseType;
    Element() : value(0), min(0), max(0), bitsz(0)
    {
    }

    void set(Type value, double min, double max, int bitsz)
    {
      this->value = value;
      this->min = min;
      this->max = max;
      this->bitsz = bitsz;
    }

    Type value;
    double min;
    double max;
    int bitsz;
  };

  virtual void SetUp()
  {
    bitval.set(0, 0, 1, 1);
    byteval.set(235, 0, 255, 8);
    intval.set(645, 0, 1023, 10);
    zdoubleval.set(0, -1, 1 - (2.0 / 512.0), 9);
    nzdoubleval.set(0.75390625, -1, 1 - (2.0 / 512.0), 9);

    bitval_encoded.push_back(bitval.value);
    byteval_encoded.push_back(byteval.value);
    intval_encoded.push_back(161);
    intval_encoded.push_back(1);
    zdoubleval_encoded.push_back(128);
    zdoubleval_encoded.push_back(0);
    nzdoubleval_encoded.push_back(224);
    nzdoubleval_encoded.push_back(1);

    multival_encoded.push_back(234);
    multival_encoded.push_back(225);
    multival_encoded.push_back(161);
    multival_encoded.push_back(5);
  }

  virtual void TearDown()
  {
  }

  template <class Type>
  void testEncoding(BitStorage& storage, Element<Type> val,
                    std::vector<uint8_t>& val_encoded)
  {
    storage.put(val.value, val.min, val.max, val.bitsz);
    EXPECT_EQ(val_encoded.size(), storage.storage().size());

    for (int i = 0; i < val_encoded.size(); ++i)
      EXPECT_EQ(val_encoded[i], storage.storage()[i]);
  }

  template <class Type>
  void testDecoding(BitStorage& storage, Element<Type> val)
  {
    Type decoded(-1);
    storage.get(decoded, val.min, val.max, val.bitsz);
    EXPECT_EQ(val.value, decoded);
  }

  Element<uint8_t> bitval;
  Element<uint8_t> byteval;
  Element<int> intval;
  Element<double> zdoubleval;
  Element<double> nzdoubleval;

  std::vector<uint8_t> bitval_encoded;
  std::vector<uint8_t> byteval_encoded;
  std::vector<uint8_t> intval_encoded;
  std::vector<uint8_t> zdoubleval_encoded;
  std::vector<uint8_t> nzdoubleval_encoded;
  std::vector<uint8_t> multival_encoded;
};

TEST_F(BitStorageTest, bitEncoding)
{
  BitStorage storage;
  testEncoding(storage, bitval, bitval_encoded);
}

TEST_F(BitStorageTest, byteEncoding)
{
  BitStorage storage;
  testEncoding(storage, byteval, byteval_encoded);
}

TEST_F(BitStorageTest, intEncoding)
{
  BitStorage storage;
  testEncoding(storage, intval, intval_encoded);
}

TEST_F(BitStorageTest, zeroDoubleEncoding)
{
  BitStorage storage;
  testEncoding(storage, zdoubleval, zdoubleval_encoded);
}

TEST_F(BitStorageTest, nonzeroDoubleEncoding)
{
  BitStorage storage;
  testEncoding(storage, nzdoubleval, nzdoubleval_encoded);
}

TEST_F(BitStorageTest, multiValueEncoding)
{
  BitStorage storage;
  storage.put(bitval.value, bitval.min, bitval.max, bitval.bitsz);
  storage.put(byteval.value, byteval.min, byteval.max, byteval.bitsz);
  storage.put(nzdoubleval.value, nzdoubleval.min, nzdoubleval.max,
              nzdoubleval.bitsz);
  storage.put(intval.value, intval.min, intval.max, intval.bitsz);
  EXPECT_EQ(multival_encoded.size(), storage.storage().size());

  for (int i = 0; i < multival_encoded.size(); ++i)
    EXPECT_EQ(multival_encoded[i], storage.storage()[i]);
}

TEST_F(BitStorageTest, bitDecoding)
{
  BitStorage storage(bitval_encoded);
  testDecoding(storage, bitval);
}

TEST_F(BitStorageTest, byteDecoding)
{
  BitStorage storage(byteval_encoded);
  testDecoding(storage, byteval);
}

TEST_F(BitStorageTest, intDecoding)
{
  BitStorage storage(intval_encoded);
  testDecoding(storage, intval);
}

TEST_F(BitStorageTest, zeroDoubleDecoding)
{
  BitStorage storage(zdoubleval_encoded);
  testDecoding(storage, zdoubleval);
}

TEST_F(BitStorageTest, nonZeroDoubleDecoding)
{
  BitStorage storage(nzdoubleval_encoded);
  testDecoding(storage, nzdoubleval);
}

TEST_F(BitStorageTest, multiValueDecoding)
{
  BitStorage storage(multival_encoded);
  testDecoding(storage, bitval);
  testDecoding(storage, byteval);
  testDecoding(storage, nzdoubleval);
  testDecoding(storage, intval);
}

TEST_F(BitStorageTest, zeroInputbits)
{
  BitStorage storage;
  storage.put(bitval.value, bitval.min, bitval.max, 0);
  EXPECT_EQ(0, storage.storage().size()) << "Storage size not 0 bytes.";
}
