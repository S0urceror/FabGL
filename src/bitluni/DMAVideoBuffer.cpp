#include "DMAVideoBuffer.h"

namespace bitluni {

DMAVideoBuffer::DMAVideoBuffer(int lines, int lineSize, int clones, bool ring, bool psram, int bufferCount)
{
	this->lineSize = lineSize;
	this->psram = psram;
	this->bufferCount = bufferCount;
	this->lines = lines;
	this->clones = clones;
	valid = false;
	descriptorCount = lines * clones; //assume we dont need more than 4095 bytes per line

	// allocate descriptors
	descriptors = (dma_descriptor_t *)heap_caps_aligned_calloc(ALIGNMENT_SRAM, 1, descriptorCount * sizeof(dma_descriptor_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
	if (!descriptors) 
		return;
	// initialize line buffers with every line zero.
	for(int i = 0; i < bufferCount; i++)		
		for(int j = 0; j < lines; j++)
			buffer[i][j] = 0;
	// allocate line buffers
	for(int i = 0; i < bufferCount; i++)
	{
		for(int y = 0; y < lines; y++)
		{
			if(psram)
				buffer[i][y] = heap_caps_aligned_calloc(ALIGNMENT_PSRAM, 1, lineSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
			else
				buffer[i][y] = heap_caps_aligned_calloc(ALIGNMENT_SRAM, 1, lineSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
			if (!buffer[i][y]) 
			{
				ESP_LOGE ("DMAVideoBuffer","Error allocating line buffer");
				return;
			}
		}
	}
	// setup descriptors for every line
	for (int i = 0; i < descriptorCount; i++) 
	{
		descriptors[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_CPU;
		descriptors[i].dw0.suc_eof = 0;
		descriptors[i].next = &descriptors[i + 1];
		descriptors[i].dw0.size = lineSize;
		descriptors[i].dw0.length = descriptors[i].dw0.size;
	}
	// attach first buffer to DMA descriptors
	attachBuffer(0);
	// continuous DMA or EOF at end of line buffer?
	if(ring)
	{
		descriptors[descriptorCount - 1].dw0.suc_eof = 0;
		descriptors[descriptorCount - 1].next = descriptors;
	}
	else
	{
		descriptors[descriptorCount - 1].dw0.suc_eof = 1;
		descriptors[descriptorCount - 1].next = 0;
	}
	valid = true;
}

DMAVideoBuffer::~DMAVideoBuffer()
{
	if(descriptors)
		heap_caps_free(descriptors);
	for(int i = 0; i < bufferCount; i++)		
		for(int j = 0; j < lines; j++)
			if(buffer[i][j])
				heap_caps_free(buffer[i][j]);
}

void DMAVideoBuffer::attachBuffer(int b)
{
	for(int i = 0; i < lines; i++) 
		for(int j = 0; j < clones; j++)
			descriptors[i * clones + j].buffer = buffer[b][i];
}

uint8_t *DMAVideoBuffer::getLineAddr8(int y ,int b)
{
	return (uint8_t*)buffer[b][y];
}


uint16_t *DMAVideoBuffer::getLineAddr16(int y ,int b)
{
	return (uint16_t*)buffer[b][y];
}

dma_descriptor_t *DMAVideoBuffer::getDescriptor(int i) const
{
	return &descriptors[0];
}

int DMAVideoBuffer::getDescriptorCount() const
{
	return descriptorCount;
}

int DMAVideoBuffer::getLineSize() const
{
	return lineSize;
}

void DMAVideoBuffer::flush(int b)
{
	if(!psram) return;
	for(int y = 0; y < lines; y++)
		Cache_WriteBack_Addr((uint32_t)buffer[b][y], lineSize);
}

void DMAVideoBuffer::flush(int b, int y)
{
	if(!psram) return;
		Cache_WriteBack_Addr((uint32_t)buffer[b][y], lineSize);
}

/*void flush(int b, int start, int length)
{
	if(!psram) return;
	Cache_WriteBack_Addr((uint32_t)start, length);
}*/

int DMAVideoBuffer::getBufferCount() const
{
	return bufferCount;
}

bool DMAVideoBuffer::isValid() const
{
	return valid;
}

} // namespace bitluni