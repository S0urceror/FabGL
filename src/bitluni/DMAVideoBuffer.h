
#include <hal/dma_types.h>
#include "spiram.h"
#include <rom/cache.h>
#include <esp_heap_caps.h>
#include "esp_log.h"

extern int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);

class DMAVideoBuffer
{
	protected:
		int	descriptorCount;
		dma_descriptor_t *descriptors;
		static const int MAX_BUFFERS = 2;	//need to malloc this
		static const int MAX_LINES = 1024;	//need to malloc this
		void *buffer[MAX_BUFFERS][MAX_LINES];

	public:
		int bufferCount;
		bool valid;
		int lines;
		int lineSize;
		bool psram;
		int clones;

		static const int MAX_DMA_BLOCK_SIZE = 4092;
		static const int ALIGNMENT_PSRAM = 64;
		static const int ALIGNMENT_SRAM = 4;

		DMAVideoBuffer(int lines, int lineSize, int clones = 1, bool ring = true, bool psram = true, int bufferCount = 1);
		~DMAVideoBuffer();

		void attachBuffer(int b = 0);
		uint8_t *getLineAddr8(int y ,int b = 0);
		uint16_t *getLineAddr16(int y ,int b = 0);

		dma_descriptor_t *getDescriptor(int i = 0) const;
		int getDescriptorCount() const;
		int getLineSize() const;
		void flush(int b = 0);
		void flush(int b, int y);
		int getBufferCount() const;
		bool isValid() const;
};