/*
 * Portions Copyright (c) 2022 EdgeImpulse Inc.
 * Portions Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dmic_sample);

// Edge Impulse SDK
#include <project_inferencing.h>
#include "edge-impulse-sdk/dsp/numpy.hpp"


#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     1000

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 1)
#define BLOCK_COUNT      5
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);


/* Audio buffer for Edge Impulse inference */
static int16_t inference_buffer[EI_CLASSIFIER_RAW_SAMPLE_COUNT];

/**
 * @brief      Copy raw audio signal data to float buffer
 *             This is a callback function for the Edge Impulse SDK
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference_buffer[offset], out_ptr, length);
    return 0;
}


static int do_pdm_transfer(const struct device *dmic_dev,
			   struct dmic_cfg *cfg,
			   size_t block_count)
{
	int ret;

	LOG_INF("PCM output rate: %u, channels: %u",
		cfg->streams[0].pcm_rate, cfg->channel.req_num_chan);

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("START trigger failed: %d", ret);
		return ret;
	}

	for (int i = 0; i < block_count; ++i) {
		void *buffer;
		uint32_t size;

		ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("%d - read failed: %d", i, ret);
			return ret;
		}

		// LOG_INF("%d - got buffer %p of %u bytes", i, buffer, size);

        // Copy data to the inference buffer
        memcpy(&inference_buffer[i * (size / BYTES_PER_SAMPLE)], buffer, size);

		k_mem_slab_free(&mem_slab, buffer);
	}

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		LOG_ERR("STOP trigger failed: %d", ret);
		return ret;
	}

	return ret;
}

int main(void)
{
	const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	int ret;

	LOG_INF("Edge Impulse Inferencing Demo (Zephyr)\n");

	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("%s is not ready", dmic_dev->name);
		return 0;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* These fields can be used to limit the PDM clock
			 * configurations that the driver is allowed to use
			 * to those supported by the microphone.
			 */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size =
		BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);


	// Summary of inferencing settings
    printk("Inferencing settings:\n");
    printk("\tInterval: %.4f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    printk("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    printk("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT * 1000 / EI_CLASSIFIER_FREQUENCY);
    printk("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));


	ret = dmic_configure(dmic_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure the driver: %d", ret);
		return ret;
	}

    while (1) {
        // printk("Starting inferencing in 2 seconds...\n");
        // k_msleep(2000);
			
		ret = do_pdm_transfer(dmic_dev, &cfg, 2 * BLOCK_COUNT);
		if (ret < 0) {
			return 0;
		}

		// Create a signal object for the Edge Impulse SDK
		signal_t signal;
		signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
		signal.get_data = &microphone_audio_signal_get_data;

		// Run the classifier
		ei_impulse_result_t result = {0};
		EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

		if (res != EI_IMPULSE_OK) {
			printk("ERR: Failed to run classifier (%d)\n", res);
		}

		// Print the predictions
		printk("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\n",
				result.timing.dsp, result.timing.classification, result.timing.anomaly);
		for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
			printk("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
		}
	}

	LOG_INF("Exiting");
	return 0;
}
